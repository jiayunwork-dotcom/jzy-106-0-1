package api

import (
	"errors"
	"net/http"

	"github.com/gin-gonic/gin"

	"attitude-service/internal/euler"
	"attitude-service/internal/integrator"
	"attitude-service/internal/quaternion"
	"attitude-service/internal/store"
	"attitude-service/internal/validation"
)

// Handler 持有接口层依赖（命名序列存储）。积分内核本身无状态，
// 每次请求独立推进，天然保证并发请求之间互不影响。
type Handler struct {
	seqs store.Store
}

// NewHandler 创建接口层。
func NewHandler(seqs store.Store) *Handler {
	return &Handler{seqs: seqs}
}

// Health 是存活探针。
func (h *Handler) Health(c *gin.Context) {
	c.JSON(http.StatusOK, gin.H{"status": "ok"})
}

// Integrate 处理 POST /api/v1/attitude/integrate。
func (h *Handler) Integrate(c *gin.Context) {
	var req integrateRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		writeError(c, http.StatusBadRequest, "INVALID_JSON",
			"请求体不是合法 JSON 或字段类型不匹配："+err.Error())
		return
	}
	raw, err := toRawInput(req)
	if err != nil {
		writeError(c, http.StatusBadRequest, "INVALID_PARAMETER", err.Error())
		return
	}
	input, err := validation.Validate(raw)
	if err != nil {
		writeError(c, http.StatusBadRequest, "INVALID_PARAMETER", err.Error())
		return
	}
	h.runAndRespond(c, input)
}

// SaveSequence 处理 POST /api/v1/sequences?name=...。
func (h *Handler) SaveSequence(c *gin.Context) {
	name := c.Query("name")
	if err := validation.NameRule(name); err != nil {
		writeError(c, http.StatusBadRequest, "INVALID_PARAMETER", err.Error())
		return
	}
	var req sequenceSaveRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		writeError(c, http.StatusBadRequest, "INVALID_JSON",
			"请求体不是合法 JSON 或字段类型不匹配："+err.Error())
		return
	}
	// 复用同一套校验：把序列当作“从恒等姿态出发”的输入来验证形状。
	raw, err := toRawInput(integrateRequest{
		InitialQuaternion: [4]float64{1, 0, 0, 0},
		AngularVelocities: req.AngularVelocities,
		Timestamps:        req.Timestamps,
		TimeSteps:         req.TimeSteps,
	})
	if err != nil {
		writeError(c, http.StatusBadRequest, "INVALID_PARAMETER", err.Error())
		return
	}
	input, err := validation.Validate(raw)
	if err != nil {
		writeError(c, http.StatusBadRequest, "INVALID_PARAMETER", err.Error())
		return
	}
	seq := store.Sequence{Name: name, Steps: input.Steps}
	if err := h.seqs.Save(seq); err != nil {
		writeError(c, http.StatusBadRequest, "INVALID_PARAMETER", err.Error())
		return
	}
	saved, _ := h.seqs.Get(name)
	c.JSON(http.StatusCreated, sequenceSummary{
		Name: saved.Name, NumSamples: saved.NumSamples, Duration: saved.Duration,
	})
}

// ListSequences 处理 GET /api/v1/sequences。
func (h *Handler) ListSequences(c *gin.Context) {
	list := h.seqs.List()
	out := make([]sequenceSummary, 0, len(list))
	for _, s := range list {
		out = append(out, sequenceSummary{
			Name: s.Name, NumSamples: s.NumSamples, Duration: s.Duration,
		})
	}
	c.JSON(http.StatusOK, gin.H{"sequences": out})
}

// GetSequence 处理 GET /api/v1/sequences/:name。
func (h *Handler) GetSequence(c *gin.Context) {
	seq, err := h.seqs.Get(c.Param("name"))
	if err != nil {
		writeError(c, http.StatusNotFound, "SEQUENCE_NOT_FOUND", err.Error())
		return
	}
	omegas := make([][3]float64, len(seq.Steps))
	dts := make([]float64, len(seq.Steps))
	for i, s := range seq.Steps {
		omegas[i] = [3]float64{s.Omega.X, s.Omega.Y, s.Omega.Z}
		dts[i] = s.Dt
	}
	c.JSON(http.StatusOK, sequenceDetail{
		sequenceSummary: sequenceSummary{
			Name: seq.Name, NumSamples: seq.NumSamples, Duration: seq.Duration,
		},
		AngularVelocities: omegas,
		TimeSteps:         dts,
	})
}

// DeleteSequence 处理 DELETE /api/v1/sequences/:name。
func (h *Handler) DeleteSequence(c *gin.Context) {
	if err := h.seqs.Delete(c.Param("name")); err != nil {
		writeError(c, http.StatusNotFound, "SEQUENCE_NOT_FOUND", err.Error())
		return
	}
	c.Status(http.StatusNoContent)
}

// IntegrateSequence 处理 POST /api/v1/sequences/:name/integrate。
func (h *Handler) IntegrateSequence(c *gin.Context) {
	seq, err := h.seqs.Get(c.Param("name"))
	if err != nil {
		writeError(c, http.StatusNotFound, "SEQUENCE_NOT_FOUND", err.Error())
		return
	}
	var req sequenceRunRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		writeError(c, http.StatusBadRequest, "INVALID_JSON",
			"请求体不是合法 JSON 或字段类型不匹配："+err.Error())
		return
	}
	// 存储里的 Steps 已经过校验，这里只校验初始姿态与配置。
	raw := validation.RawInput{
		Q0:           req.InitialQuaternion,
		Method:       req.Method,
		WarnDrift:    req.WarnNormDrift,
		RejectDrift:  req.RejectNormDrift,
		IncludeTrace: req.IncludeTrace,
	}
	raw.Omegas = make([][3]float64, len(seq.Steps))
	raw.Dts = make([]float64, len(seq.Steps))
	for i, s := range seq.Steps {
		raw.Omegas[i] = [3]float64{s.Omega.X, s.Omega.Y, s.Omega.Z}
		raw.Dts[i] = s.Dt
	}
	input, err := validation.Validate(raw)
	if err != nil {
		writeError(c, http.StatusBadRequest, "INVALID_PARAMETER", err.Error())
		return
	}
	h.runAndRespond(c, input)
}

// runAndRespond 驱动积分内核并组装响应；积分被阈值拒绝时返回 422。
func (h *Handler) runAndRespond(c *gin.Context, input *validation.Input) {
	res, err := integrator.Integrate(input.Q0, input.Steps, input.Config)
	if err != nil {
		var rej *integrator.StepRejectedError
		if errors.As(err, &rej) {
			writeError(c, http.StatusUnprocessableEntity, "STEP_NORM_DRIFT_EXCEEDED", rej.Error())
			return
		}
		writeError(c, http.StatusBadRequest, "INTEGRATION_FAILED", err.Error())
		return
	}
	c.JSON(http.StatusOK, buildResponse(res))
}

func buildResponse(res *integrator.Result) integrateResponse {
	out := integrateResponse{
		FinalQuaternion:  [4]float64{res.Final.W, res.Final.X, res.Final.Y, res.Final.Z},
		FinalEuler:       toEulerResponse(euler.FromQuaternion(res.Final)),
		Steps:            res.Steps,
		Duration:         res.Duration,
		FinalNorm:        res.FinalNorm,
		FinalNormDrift:   res.FinalNormDrift,
		MaxStepNormDrift: res.MaxStepDrift,
	}
	for _, w := range res.Warnings {
		out.Warnings = append(out.Warnings, warningResponse{
			Step: w.Step, Dt: w.Dt, NormDrift: w.Drift,
		})
	}
	if out.Warnings == nil {
		out.Warnings = []warningResponse{}
	}
	for _, tp := range res.Trace {
		q := quatFromArray(tp.Quat)
		out.Trace = append(out.Trace, tracePointResponse{
			T:          tp.T,
			Quaternion: tp.Quat,
			Euler:      toEulerResponse(euler.FromQuaternion(q)),
		})
	}
	return out
}

func toEulerResponse(r euler.Result) eulerResponse {
	return eulerResponse{
		Roll: r.Roll, Pitch: r.Pitch, Yaw: r.Yaw,
		Convention: r.Convention, Singular: r.Singular, Warning: r.Warning,
	}
}

func quatFromArray(a [4]float64) quaternion.Quat {
	return quaternion.Quat{W: a[0], X: a[1], Y: a[2], Z: a[3]}
}

func writeError(c *gin.Context, status int, code, msg string) {
	c.JSON(status, errorResponse{Error: errorBody{Code: code, Message: msg}})
}

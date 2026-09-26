// Package api 是 HTTP 接口层：只负责解析 JSON、调用校验与积分内核、
// 组织响应。所有数学都在 internal/quaternion、internal/integrator、
// internal/euler 中。
package api

// integrateRequest 是 POST /api/v1/attitude/integrate 的请求体。
// timestamps（长度 N+1）与 time_steps（长度 N）二选一。
type integrateRequest struct {
	InitialQuaternion [4]float64  `json:"initial_quaternion"`
	AngularVelocities [][]float64 `json:"angular_velocities"`
	Timestamps        []float64   `json:"timestamps"`
	TimeSteps         []float64   `json:"time_steps"`
	Method            string      `json:"method"`
	WarnNormDrift     float64     `json:"warn_norm_drift"`
	RejectNormDrift   float64     `json:"reject_norm_drift"`
	IncludeTrace      bool        `json:"include_trace"`
}

// sequenceSaveRequest 是 POST /api/v1/sequences 的请求体。
type sequenceSaveRequest struct {
	AngularVelocities [][]float64 `json:"angular_velocities"`
	Timestamps        []float64   `json:"timestamps"`
	TimeSteps         []float64   `json:"time_steps"`
}

// sequenceRunRequest 是 POST /api/v1/sequences/:name/integrate 的请求体：
// 序列来自存储，这里只给初始姿态与积分配置。
type sequenceRunRequest struct {
	InitialQuaternion [4]float64 `json:"initial_quaternion"`
	Method            string     `json:"method"`
	WarnNormDrift     float64    `json:"warn_norm_drift"`
	RejectNormDrift   float64    `json:"reject_norm_drift"`
	IncludeTrace      bool       `json:"include_trace"`
}

type errorBody struct {
	Code    string `json:"code"`
	Message string `json:"message"`
}

type errorResponse struct {
	Error errorBody `json:"error"`
}

type eulerResponse struct {
	Roll       float64 `json:"roll"`
	Pitch      float64 `json:"pitch"`
	Yaw        float64 `json:"yaw"`
	Convention string  `json:"convention"`
	Singular   bool    `json:"singular"`
	Warning    string  `json:"warning,omitempty"`
}

type tracePointResponse struct {
	T          float64       `json:"t"`
	Quaternion [4]float64    `json:"quaternion"`
	Euler      eulerResponse `json:"euler"`
}

type integrateResponse struct {
	FinalQuaternion  [4]float64           `json:"final_quaternion"`
	FinalEuler       eulerResponse        `json:"final_euler"`
	Steps            int                  `json:"steps"`
	Duration         float64              `json:"duration"`
	FinalNorm        float64              `json:"final_norm"`
	FinalNormDrift   float64              `json:"final_norm_drift"`
	MaxStepNormDrift float64              `json:"max_step_norm_drift"`
	Warnings         []warningResponse    `json:"warnings"`
	Trace            []tracePointResponse `json:"trace,omitempty"`
}

type warningResponse struct {
	Step      int     `json:"step"`
	Dt        float64 `json:"dt"`
	NormDrift float64 `json:"norm_drift"`
}

type sequenceSummary struct {
	Name       string  `json:"name"`
	NumSamples int     `json:"num_samples"`
	Duration   float64 `json:"duration"`
}

type sequenceDetail struct {
	sequenceSummary
	AngularVelocities [][3]float64 `json:"angular_velocities"`
	TimeSteps         []float64    `json:"time_steps"`
}

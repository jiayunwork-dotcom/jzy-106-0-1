// Package integrator 是姿态四元数的时间推进内核。
//
// 四元数代数（乘法/归一化/共轭）在 internal/quaternion，
// 本包只负责沿“角速度采样 + 每采样时间步长”的序列推进姿态。
//
// 运动学方程（机体角速度，body frame）：
//
//	dq/dt = 1/2 · q ⊗ (0, ω_b)        —— 机体系角速度右乘
//
// 支持两种推进方法：
//   - MethodRK4：四阶经典龙格-库塔，每步推进后重新归一化；
//   - MethodExp：指数映射（零阶保持），每步 q ↦ q ⊗ exp(ω dt/2)，
//     本身即保范，再做一次归一化兜底。
package integrator

import (
	"errors"
	"fmt"
	"math"

	"attitude-service/internal/quaternion"
)

// 支持的推进方法。
const (
	MethodRK4 = "rk4"
	MethodExp = "exp"
)

// ErrUnknownMethod 表示请求了不支持的积分方法。
var ErrUnknownMethod = errors.New("未知积分方法，仅支持 rk4 / exp")

// StepRejectedError 表示某一步单步范数漂移超过拒绝阈值，
// 积分被中止；Step 为基于 1 的步序号。
type StepRejectedError struct {
	Step      int
	Dt        float64
	Norm      float64
	Drift     float64
	Threshold float64
}

func (e *StepRejectedError) Error() string {
	return fmt.Sprintf(
		"第 %d 步（dt=%.6g s）单步范数漂移 |1-|q||=%.3g 超过拒绝阈值 %.3g，已拒绝继续积分（请减小时间步长）",
		e.Step, e.Dt, e.Drift, e.Threshold)
}

// Config 控制积分过程。零值字段由 Validate（或 Integrate）填默认值。
type Config struct {
	Method          string  // "rk4"（默认）或 "exp"
	WarnNormDrift   float64 // 单步漂移（归一化前）超过该值则告警，默认 1e-6
	RejectNormDrift float64 // 单步漂移超过该值则拒绝积分，默认 1e-2
	IncludeTrace    bool    // 是否返回每个时间点的中间姿态
}

// Sample 是一个角速度采样：ω 为该采样区间内的机体系角速度 (rad/s)，
// Dt 为该采样自带的时间步长 (s)。
type Sample struct {
	Omega quaternion.Vec3
	Dt    float64
}

// TracePoint 是某一时间点上的中间姿态。
type TracePoint struct {
	T            float64    `json:"t"`
	Quat         [4]float64 `json:"quaternion"`
	PreNorm      float64    `json:"pre_norm_quaternion,omitempty"`
	PreNormDrift float64    `json:"pre_norm_drift,omitempty"`
}

// Warning 记录一次单步范数漂移告警（未达到拒绝阈值）。
type Warning struct {
	Step  int     `json:"step"`
	Dt    float64 `json:"dt"`
	Drift float64 `json:"norm_drift"`
}

// Result 是一次完整推进的结果。
type Result struct {
	Final          quaternion.Quat // 末端姿态（已归一化）
	Steps          int             // 实际推进步数
	Duration       float64         // 总时长 (s)
	FinalNorm      float64         // 末端归一化后模长（应恒为 1）
	FinalNormDrift float64         // |末端模长 - 1|（归一化后）
	MaxStepDrift   float64         // 全程最大单步漂移（归一化前）
	Warnings       []Warning       // 单步漂移告警
	Trace          []TracePoint    // 长度 Steps+1，IncludeTrace=true 时填充
}

// Integrate 从 q0 出发，按 cfg 沿 samples 推进姿态。
// q0 必须是单位四元数、samples 非空且 Dt>0（合法性校验见 internal/validation，
// 此处只兜底）。任何一步漂移超拒绝阈值即返回 *StepRejectedError，且不会
// 返回部分结果。
func Integrate(q0 quaternion.Quat, samples []Sample, cfg Config) (*Result, error) {
	if cfg.Method == "" {
		cfg.Method = MethodRK4
	}
	if cfg.WarnNormDrift <= 0 {
		cfg.WarnNormDrift = 1e-6
	}
	if cfg.RejectNormDrift <= 0 {
		cfg.RejectNormDrift = 1e-2
	}
	if len(samples) == 0 {
		return nil, errors.New("角速度序列为空")
	}
	if n := q0.Norm(); math.Abs(n-1) > 1e-6 {
		return nil, fmt.Errorf("初始四元数不是单位四元数：|q|=%.9g", n)
	}

	q, ok := q0.Normalized()
	if !ok {
		return nil, errors.New("初始四元数无法归一化")
	}

	res := &Result{Steps: len(samples)}
	t := 0.0
	if cfg.IncludeTrace {
		res.Trace = make([]TracePoint, 0, len(samples)+1)
		res.Trace = append(res.Trace, tracePoint(t, q, 1))
	}

	for i, s := range samples {
		if !math.IsNaN(s.Dt) && !math.IsInf(s.Dt, 0) && s.Dt <= 0 {
			return nil, fmt.Errorf("第 %d 个时间步长非正：%.6g", i+1, s.Dt)
		}
		qStep, preNorm := step(q, s.Omega, s.Dt, cfg.Method)
		drift := math.Abs(preNorm - 1)
		if drift > cfg.RejectNormDrift {
			return nil, &StepRejectedError{
				Step: i + 1, Dt: s.Dt, Norm: preNorm,
				Drift: drift, Threshold: cfg.RejectNormDrift,
			}
		}
		if drift > cfg.WarnNormDrift {
			res.Warnings = append(res.Warnings, Warning{
				Step: i + 1, Dt: s.Dt, Drift: drift,
			})
		}
		if drift > res.MaxStepDrift {
			res.MaxStepDrift = drift
		}

		var normalized bool
		q, normalized = qStep.Normalized()
		if !normalized {
			return nil, fmt.Errorf("第 %d 步后四元数退化（模长为 0/非有限），无法归一化", i+1)
		}
		t += s.Dt
		res.Duration = t
		if cfg.IncludeTrace {
			res.Trace = append(res.Trace, tracePoint(t, q, preNorm))
		}
	}

	res.Final = q
	res.FinalNorm = q.Norm()
	res.FinalNormDrift = math.Abs(res.FinalNorm - 1)
	return res, nil
}

// derivative 返回运动学导数 f(q) = 1/2 q ⊗ (0, ω)。
// 注意 ω 是机体系角速度，因此是右乘（若为参考系角速度才是左乘）。
func derivative(q quaternion.Quat, w quaternion.Vec3) quaternion.Quat {
	return quaternion.Mul(q, quaternion.Pure(w)).Scale(0.5)
}

// step 推进一个时间步（区间内角速度保持为 ω），返回归一化前四元数与模长。
func step(q quaternion.Quat, w quaternion.Vec3, dt float64, method string) (quaternion.Quat, float64) {
	var qNext quaternion.Quat
	switch method {
	case MethodRK4:
		k1 := derivative(q, w)
		k2 := derivative(quaternion.Add(q, k1.Scale(dt/2)), w)
		k3 := derivative(quaternion.Add(q, k2.Scale(dt/2)), w)
		k4 := derivative(quaternion.Add(q, k3.Scale(dt)), w)
		inc := quaternion.Add(
			k1.Scale(dt/6),
			quaternion.Add(k2.Scale(2*dt/6),
				quaternion.Add(k3.Scale(2*dt/6), k4.Scale(dt/6))))
		qNext = quaternion.Add(q, inc)
	case MethodExp:
		// q(t+dt) = q(t) ⊗ exp((0, ω dt)/2)，机体系角速度右乘
		qNext = quaternion.Mul(q, quaternion.ExpRotation(quaternion.ScaleVec(w, dt)))
	default:
		panic(ErrUnknownMethod)
	}
	return qNext, qNext.Norm()
}

func tracePoint(t float64, q quaternion.Quat, preNorm float64) TracePoint {
	return TracePoint{
		T:            t,
		Quat:         [4]float64{q.W, q.X, q.Y, q.Z},
		PreNorm:      preNorm,
		PreNormDrift: math.Abs(preNorm - 1),
	}
}

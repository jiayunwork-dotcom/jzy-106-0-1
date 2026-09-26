// Package validation 在积分开始之前集中拦截非法请求参数。
// 它只描述“合法的输入长什么样”，不触碰 HTTP（解析在 internal/api），
// 也不包含任何积分数学。
package validation

import (
	"errors"
	"fmt"
	"math"

	"attitude-service/internal/integrator"
	"attitude-service/internal/quaternion"
)

// 输入模式：给“时间戳”或给“每步 dt”二选一。
const (
	ModeTimestamps = "timestamps" // omegas 长度 N，timestamps 长度 N+1 且严格递增
	ModeDts        = "dts"        // omegas 长度 N，dts 长度 N
)

// Input 是校验通过后的、与传输格式无关的积分输入。
type Input struct {
	Mode  string
	Q0    quaternion.Quat
	Steps []integrator.Sample
	// Timestamps 仅 ModeTimestamps 下填充，长度 N+1。
	Timestamps []float64
	// Config 已填好默认值并经过检查。
	Config integrator.Config
}

// FieldError 指出具体哪个参数、为什么不合法。
type FieldError struct {
	Field  string
	Reason string
}

func (e *FieldError) Error() string {
	return fmt.Sprintf("参数 %s 非法：%s", e.Field, e.Reason)
}

func fail(field, format string, args ...any) error {
	return &FieldError{Field: field, Reason: fmt.Sprintf(format, args...)}
}

// RawInput 是 API 层转译后交给校验器的原始数据。
// Omegas[i] 必须是长度 3 的向量；Timestamps / Dts 二选一非空。
type RawInput struct {
	Q0           [4]float64
	Omegas       [][3]float64
	Timestamps   []float64
	Dts          []float64
	Method       string
	WarnDrift    float64 // <=0 表示用默认值
	RejectDrift  float64
	IncludeTrace bool
}

// Validate 执行全部前置校验，返回可直接喂给 integrator 的 Input。
func Validate(in RawInput) (*Input, error) {
	// ---- 初始四元数 ----
	if !isFinite4(in.Q0) {
		return nil, fail("initial_quaternion", "分量必须全部为有限实数，不能是 NaN/Inf")
	}
	q0 := quaternion.Quat{W: in.Q0[0], X: in.Q0[1], Y: in.Q0[2], Z: in.Q0[3]}
	n := q0.Norm()
	if math.Abs(n-1) > 1e-6 {
		return nil, fail("initial_quaternion",
			"必须是单位四元数，当前模长 |q|=%.9g（请归一化后重试）", n)
	}

	// ---- 序列非空 ----
	if len(in.Omegas) == 0 {
		return nil, fail("angular_velocities", "角速度序列为空：至少需要一个采样")
	}
	for i, w := range in.Omegas {
		if !isFinite3(w) {
			return nil, fail(fmt.Sprintf("angular_velocities[%d]", i),
				"角速度分量必须全部为有限实数，不能是 NaN/Inf")
		}
	}

	// ---- 时间模式：timestamps 与 dts 二选一 ----
	mode := ""
	switch {
	case len(in.Timestamps) > 0 && len(in.Dts) > 0:
		return nil, fail("timestamps", "timestamps 与 time_steps 二选一，不能同时提供")
	case len(in.Timestamps) > 0:
		mode = ModeTimestamps
	case len(in.Dts) > 0:
		mode = ModeDts
	default:
		return nil, errors.New("缺少时间信息：必须提供 timestamps（长度 N+1）或 time_steps（长度 N）之一")
	}

	steps := make([]integrator.Sample, len(in.Omegas))
	var timestamps []float64

	switch mode {
	case ModeTimestamps:
		if len(in.Timestamps) != len(in.Omegas)+1 {
			return nil, fail("timestamps",
				"时间戳个数必须比角速度采样多 1（共 N+1 个，N 个角速度采样）：角速度 %d 个，时间戳 %d 个",
				len(in.Omegas), len(in.Timestamps))
		}
		for i, ts := range in.Timestamps {
			if math.IsNaN(ts) || math.IsInf(ts, 0) {
				return nil, fail(fmt.Sprintf("timestamps[%d]", i), "时间戳必须是有限实数")
			}
		}
		for i := 0; i < len(in.Omegas); i++ {
			dt := in.Timestamps[i+1] - in.Timestamps[i]
			if dt <= 0 {
				return nil, fail(fmt.Sprintf("timestamps[%d]", i+1),
					"时间戳必须严格递增：t[%d]-t[%d] = %.6g 非正", i+1, i, dt)
			}
			steps[i] = integrator.Sample{
				Omega: quaternion.Vec3{X: in.Omegas[i][0], Y: in.Omegas[i][1], Z: in.Omegas[i][2]},
				Dt:    dt,
			}
		}
		timestamps = append(timestamps, in.Timestamps...)
	case ModeDts:
		if len(in.Dts) != len(in.Omegas) {
			return nil, fail("time_steps",
				"时间步长个数必须与角速度采样个数一致：角速度 %d 个，时间步长 %d 个",
				len(in.Omegas), len(in.Dts))
		}
		for i, dt := range in.Dts {
			if math.IsNaN(dt) || math.IsInf(dt, 0) {
				return nil, fail(fmt.Sprintf("time_steps[%d]", i), "时间步长必须是有限实数")
			}
			if dt <= 0 {
				return nil, fail(fmt.Sprintf("time_steps[%d]", i), "时间步长必须为正，当前值 %.6g", dt)
			}
		}
		for i := 0; i < len(in.Omegas); i++ {
			steps[i] = integrator.Sample{
				Omega: quaternion.Vec3{X: in.Omegas[i][0], Y: in.Omegas[i][1], Z: in.Omegas[i][2]},
				Dt:    in.Dts[i],
			}
		}
	}

	// ---- 方法与阈值 ----
	method := in.Method
	if method == "" {
		method = integrator.MethodRK4
	}
	if method != integrator.MethodRK4 && method != integrator.MethodExp {
		return nil, fail("method", "仅支持 rk4（四阶龙格库塔）或 exp（指数映射），收到 %q", method)
	}
	warn, reject := in.WarnDrift, in.RejectDrift
	if warn == 0 {
		warn = 1e-6
	}
	if reject == 0 {
		reject = 1e-2
	}
	if warn <= 0 || math.IsNaN(warn) || math.IsInf(warn, 0) {
		return nil, fail("warn_norm_drift", "告警阈值必须为正有限值")
	}
	if reject <= 0 || math.IsNaN(reject) || math.IsInf(reject, 0) {
		return nil, fail("reject_norm_drift", "拒绝阈值必须为正有限值")
	}
	if warn > reject {
		return nil, fail("warn_norm_drift", "告警阈值 %.3g 不能大于拒绝阈值 %.3g", warn, reject)
	}

	return &Input{
		Mode:       mode,
		Q0:         q0,
		Steps:      steps,
		Timestamps: timestamps,
		Config: integrator.Config{
			Method:          method,
			WarnNormDrift:   warn,
			RejectNormDrift: reject,
			IncludeTrace:    in.IncludeTrace,
		},
	}, nil
}

func isFinite3(v [3]float64) bool {
	for _, x := range v {
		if math.IsNaN(x) || math.IsInf(x, 0) {
			return false
		}
	}
	return true
}

func isFinite4(v [4]float64) bool {
	for _, x := range v {
		if math.IsNaN(x) || math.IsInf(x, 0) {
			return false
		}
	}
	return true
}

// NameRule 校验命名序列的名字：非空、长度 1..64、只允许字母数字 _-。
func NameRule(name string) error {
	if name == "" {
		return fail("name", "名称不能为空")
	}
	if len(name) > 64 {
		return fail("name", "名称长度不能超过 64 字节，当前 %d 字节", len(name))
	}
	for _, r := range name {
		if !(r >= 'a' && r <= 'z' || r >= 'A' && r <= 'Z' ||
			r >= '0' && r <= '9' || r == '_' || r == '-') {
			return fail("name", "只允许字母、数字、下划线与连字符，收到非法字符 %q", r)
		}
	}
	return nil
}

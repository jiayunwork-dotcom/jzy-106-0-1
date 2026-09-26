package api

import (
	"fmt"

	"attitude-service/internal/validation"
)

// toRawInput 把传输层请求转译为校验器输入，并在此处完成
// “每个角速度必须是长度 3 的向量”这一形状检查。
func toRawInput(req integrateRequest) (validation.RawInput, error) {
	omegas := make([][3]float64, len(req.AngularVelocities))
	for i, w := range req.AngularVelocities {
		if len(w) != 3 {
			return validation.RawInput{}, fmt.Errorf(
				"参数 angular_velocities[%d] 非法：必须是长度 3 的向量 [wx,wy,wz]，当前长度 %d", i, len(w))
		}
		omegas[i] = [3]float64{w[0], w[1], w[2]}
	}
	return validation.RawInput{
		Q0:           req.InitialQuaternion,
		Omegas:       omegas,
		Timestamps:   req.Timestamps,
		Dts:          req.TimeSteps,
		Method:       req.Method,
		WarnDrift:    req.WarnNormDrift,
		RejectDrift:  req.RejectNormDrift,
		IncludeTrace: req.IncludeTrace,
	}, nil
}

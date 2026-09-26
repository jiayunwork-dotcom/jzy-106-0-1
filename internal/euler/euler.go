// Package euler 负责单位姿态四元数与欧拉角之间的换算，以及
// 俯仰 ±90° 奇异区（gimbal lock）的检测。
//
// 全服务只支持一种约定：ZYX 内旋（intrinsic Z-Y-X），
// 即航空航天常用的 yaw-pitch-roll（ψ-θ-φ）：
//
//	R = Rz(ψ) · Ry(θ) · Rx(φ)
//
// 姿态四元数 q 把机体系向量旋转到参考系：v_ref = q ⊗ v_body ⊗ q*。
// 所有角度单位均为弧度。
package euler

import (
	"math"

	"attitude-service/internal/quaternion"
)

// ConventionZYX 是服务固定使用的欧拉角约定标识，会原样写进响应。
const ConventionZYX = "ZYX"

// DefaultSingularityTol 是奇异区判定阈值：当 |sin θ| 超过
// 1-DefaultSingularityTol 时认为姿态进入俯仰 ±90° 邻域。
const DefaultSingularityTol = 1e-9

// Angles 是一组 ZYX 内旋欧拉角（弧度）。
type Angles struct {
	Roll  float64 `json:"roll"`  // φ，绕 x 轴
	Pitch float64 `json:"pitch"` // θ，绕 y 轴
	Yaw   float64 `json:"yaw"`   // ψ，绕 z 轴
}

// Result 是换算结果：欧拉角、约定标识与奇异告警。
type Result struct {
	Angles
	Convention string `json:"convention"`
	Singular   bool   `json:"singular"`
	Warning    string `json:"warning,omitempty"`
}

// ToQuaternion 把 ZYX 内旋欧拉角换算为单位四元数：
//
//	q = qz(ψ) ⊗ qy(θ) ⊗ qx(φ)
func ToQuaternion(a Angles) quaternion.Quat {
	cr, sr := math.Cos(a.Roll/2), math.Sin(a.Roll/2)
	cp, sp := math.Cos(a.Pitch/2), math.Sin(a.Pitch/2)
	cy, sy := math.Cos(a.Yaw/2), math.Sin(a.Yaw/2)
	return quaternion.Quat{
		W: cr*cp*cy + sr*sp*sy,
		X: sr*cp*cy - cr*sp*sy,
		Y: cr*sp*cy + sr*cp*sy,
		Z: cr*cp*sy - sr*sp*cy,
	}
}

// FromQuaternion 把单位四元数换算为 ZYX 内旋欧拉角。
//
// 正常情形：
//
//	φ = atan2( 2(wx+yz),      1-2(x²+y²) )
//	θ = asin ( 2(wy-zx) )                （先裁剪到 [-1,1]）
//	ψ = atan2( 2(wz+xy),      1-2(y²+z²) )
//
// 奇异情形（|sin θ| ≥ 1-tol，即俯仰接近 ±90°）：roll 与 yaw 不可分离，
// 按常规取 φ=0，用旋转矩阵元素 R01/R11 解出 ψ，保证返回有限值而不是 NaN，
// 同时置 Singular=true 并给出 Warning。
func FromQuaternion(q quaternion.Quat) Result {
	return FromQuaternionTol(q, DefaultSingularityTol)
}

// FromQuaternionTol 与 FromQuaternion 相同，但奇异阈值可配置。
func FromQuaternionTol(q quaternion.Quat, tol float64) Result {
	// sinθ = R20 的相反数（见包注释中的矩阵约定）
	sinPitch := 2 * (q.W*q.Y - q.Z*q.X)
	if sinPitch > 1 {
		sinPitch = 1
	}
	if sinPitch < -1 {
		sinPitch = -1
	}

	res := Result{Convention: ConventionZYX}

	if math.Abs(sinPitch) >= 1-tol {
		// 奇异区：φ 置 0，ψ 由 R01/R11 解出。
		// R01 = 2(xy - wz)，R11 = 1 - 2(x²+z²)。
		r01 := 2 * (q.X*q.Y - q.W*q.Z)
		r11 := 1 - 2*(q.X*q.X+q.Z*q.Z)
		res.Roll = 0
		res.Pitch = math.Copysign(math.Pi/2, sinPitch)
		res.Yaw = math.Atan2(-r01, r11)
		res.Singular = true
		if sinPitch > 0 {
			res.Warning = "姿态处于俯仰 +90° 奇异区（gimbal lock）：roll 与 yaw 不可分离，已按 roll=0 约定输出"
		} else {
			res.Warning = "姿态处于俯仰 -90° 奇异区（gimbal lock）：roll 与 yaw 不可分离，已按 roll=0 约定输出"
		}
		return res
	}

	res.Roll = math.Atan2(2*(q.W*q.X+q.Y*q.Z), 1-2*(q.X*q.X+q.Y*q.Y))
	res.Pitch = math.Asin(sinPitch)
	res.Yaw = math.Atan2(2*(q.W*q.Z+q.X*q.Y), 1-2*(q.Y*q.Y+q.Z*q.Z))
	return res
}

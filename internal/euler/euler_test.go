package euler

import (
	"math"
	"testing"

	"attitude-service/internal/quaternion"
)

// 普通姿态：ToQuaternion → FromQuaternion 往返一致。
func TestRoundTrip(t *testing.T) {
	cases := []Angles{
		{},
		{Roll: 0.3},
		{Pitch: 0.4},
		{Yaw: 0.5},
		{Roll: -0.7, Pitch: 0.6, Yaw: 1.2},
		{Roll: 1.5, Pitch: -0.9, Yaw: -2.0},
		{Roll: -2.5, Pitch: 0.2, Yaw: 3.0},
	}
	for _, a := range cases {
		q := ToQuaternion(a)
		if math.Abs(q.Norm()-1) > 1e-14 {
			t.Fatalf("Euler→Quat 模长非 1: %v → %.15g", a, q.Norm())
		}
		got := FromQuaternion(q)
		if got.Singular {
			t.Errorf("%v 不应被判为奇异", a)
		}
		if normAngle(got.Roll-a.Roll) > 1e-11 {
			t.Errorf("roll 往返偏差 %.2e: want %.4f got %.4f",
				math.Abs(got.Roll-a.Roll), a.Roll, got.Roll)
		}
		if math.Abs(got.Pitch-a.Pitch) > 1e-11 {
			t.Errorf("pitch 往返偏差 %.2e: want %.4f got %.4f",
				math.Abs(got.Pitch-a.Pitch), a.Pitch, got.Pitch)
		}
		if normAngle(got.Yaw-a.Yaw) > 1e-11 {
			t.Errorf("yaw 往返偏差 %.2e: want %.4f got %.4f",
				math.Abs(got.Yaw-a.Yaw), a.Yaw, got.Yaw)
		}
		if got.Convention != ConventionZYX {
			t.Errorf("convention = %q", got.Convention)
		}
	}
}

// 纯轴旋转核对：只绕 z 转，必须只产生 yaw。
func TestPureAxes(t *testing.T) {
	for _, psi := range []float64{-2.3, -0.4, 0.0, 0.7, 2.1} {
		r := FromQuaternion(ToQuaternion(Angles{Yaw: psi}))
		if normAngle(r.Yaw-psi) > 1e-12 {
			t.Errorf("纯 yaw %.3f 解得 %.6f", psi, r.Yaw)
		}
		if math.Abs(r.Roll) > 1e-12 || math.Abs(r.Pitch) > 1e-12 {
			t.Errorf("纯 yaw %.3f 出现 roll/pitch: %.2e %.2e", psi, r.Roll, r.Pitch)
		}
	}
	for _, theta := range []float64{-1.2, -0.5, 0.3, 0.9} {
		r := FromQuaternion(ToQuaternion(Angles{Pitch: theta}))
		if math.Abs(r.Pitch-theta) > 1e-12 || math.Abs(r.Roll) > 1e-12 || math.Abs(r.Yaw) > 1e-12 {
			t.Errorf("纯 pitch %.3f 解得 roll=%.2e pitch=%.6f yaw=%.2e",
				theta, r.Roll, r.Pitch, r.Yaw)
		}
	}
	for _, phi := range []float64{-2.0, -0.8, 0.6, 1.3} {
		r := FromQuaternion(ToQuaternion(Angles{Roll: phi}))
		if normAngle(r.Roll-phi) > 1e-12 || math.Abs(r.Pitch) > 1e-12 || math.Abs(r.Yaw) > 1e-12 {
			t.Errorf("纯 roll %.3f 解得 roll=%.6f pitch=%.2e yaw=%.2e",
				phi, r.Roll, r.Pitch, r.Yaw)
		}
	}
}

// 奇异区：俯仰恰为 ±90°，必须显式告警、所有输出有限（不能吐 NaN），
// 并按 roll=0 约定给出 yaw：北极 yaw=ψ-φ，南极 yaw=ψ+φ。
func TestSingularityDetectedAndFinite(t *testing.T) {
	const eps = 1e-12
	cases := []struct {
		name       string
		pitch      float64
		yawAtRoll0 float64 // roll=0 时应报的 yaw
	}{
		{"north +90", math.Pi / 2, 1.9},
		{"south -90", -math.Pi / 2, 1.9},
	}
	for _, tc := range cases {
		q := ToQuaternion(Angles{Roll: 0, Pitch: tc.pitch, Yaw: tc.yawAtRoll0})
		r := FromQuaternionTol(q, eps)
		if !r.Singular {
			t.Errorf("%s: 必须标记 singular=true", tc.name)
		}
		if r.Warning == "" {
			t.Errorf("%s: 必须给出奇异告警文字", tc.name)
		}
		if !isFinite(r.Roll) || !isFinite(r.Pitch) || !isFinite(r.Yaw) {
			t.Errorf("%s: 输出含非数 roll=%.4f pitch=%.4f yaw=%.4f", tc.name, r.Roll, r.Pitch, r.Yaw)
		}
		if math.Abs(r.Roll) > 1e-12 {
			t.Errorf("%s: roll 应为 0，得到 %.4f", tc.name, r.Roll)
		}
		if normAngle(r.Yaw-tc.yawAtRoll0) > 1e-9 {
			t.Errorf("%s: yaw 应按 roll=0 约定解出 %.4f，得到 %.4f", tc.name, tc.yawAtRoll0, r.Yaw)
		}
	}
}

// 接近（但未进入）奇异区：不应告警，且仍可正确往返。
func TestNearButNotSingular(t *testing.T) {
	theta := math.Pi/2 - 1e-4
	a := Angles{Roll: 0.3, Pitch: theta, Yaw: -0.6}
	r := FromQuaternionTol(ToQuaternion(a), DefaultSingularityTol)
	if r.Singular {
		t.Errorf("距奇异区 1e-4 rad（阈值 1e-9）不应告警")
	}
	if normAngle(r.Roll-a.Roll) > 1e-9 || math.Abs(r.Pitch-a.Pitch) > 1e-9 || normAngle(r.Yaw-a.Yaw) > 1e-9 {
		t.Errorf("近奇异区往返失败: roll=%.6f pitch=%.6f yaw=%.6f", r.Roll, r.Pitch, r.Yaw)
	}
}

// 与积分器输出的交叉检查：ExpRotation(z, ψ) 必须解出 yaw=ψ。
func TestExpRotationGivesYaw(t *testing.T) {
	psi := 0.87
	q := quaternion.ExpRotation(quaternion.Vec3{Z: psi})
	r := FromQuaternion(q)
	if normAngle(r.Yaw-psi) > 1e-12 {
		t.Errorf("ExpRotation z 轴转角 %.4f 解出 yaw %.6f", psi, r.Yaw)
	}
}

func isFinite(x float64) bool {
	return !math.IsNaN(x) && !math.IsInf(x, 0)
}

// normAngle 把角度差化到 (-π, π]。
func normAngle(d float64) float64 {
	for d > math.Pi {
		d -= 2 * math.Pi
	}
	for d <= -math.Pi {
		d += 2 * math.Pi
	}
	return d
}

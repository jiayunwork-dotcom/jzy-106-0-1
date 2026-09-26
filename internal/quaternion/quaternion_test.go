package quaternion

import (
	"math"
	"testing"
)

func TestMulAssociativeAndIdentity(t *testing.T) {
	a := Quat{W: 0.5, X: 0.5, Y: 0.5, Z: 0.5}
	b := Quat{W: 0.8, X: -0.1, Y: 0.2, Z: 0.3}
	c := Quat{W: 0.9, X: 0.1, Y: -0.4, Z: 0.2}

	left := Mul(Mul(a, b), c)
	right := Mul(a, Mul(b, c))
	if d := maxDiff(left, right); d > 1e-14 {
		t.Errorf("乘法不满足结合律：偏差 %.3e", d)
	}
	if d := maxDiff(Mul(a, Identity()), a); d > 1e-15 {
		t.Errorf("右乘单位元改变了四元数：%.3e", d)
	}
	if d := maxDiff(Mul(Identity(), a), a); d > 1e-15 {
		t.Errorf("左乘单位元改变了四元数：%.3e", d)
	}
}

func TestMulPreservesUnitNorm(t *testing.T) {
	a := ExpRotation(Vec3{X: 0.3, Y: -0.2, Z: 0.5})
	b := ExpRotation(Vec3{X: -0.1, Y: 0.4, Z: 0.2})
	p := Mul(a, b)
	if math.Abs(p.Norm()-1) > 1e-14 {
		t.Errorf("单位四元数乘积模长应为 1，得到 %.17g", p.Norm())
	}
}

func TestConjugateIsInverseForUnit(t *testing.T) {
	q := ExpRotation(Vec3{X: 1, Y: 2, Z: 3})
	p := Mul(q, q.Conj())
	if d := maxDiff(p, Identity()); d > 1e-14 {
		t.Errorf("q⊗q* 应为单位元，偏差 %.3e", d)
	}
	if d := maxDiff(q.Inv(), q.Conj()); d > 1e-14 {
		t.Errorf("单位四元数的逆应等于共轭，偏差 %.3e", d)
	}
}

func TestNormalize(t *testing.T) {
	q := Quat{W: 1, X: 2, Y: 3, Z: 4}
	n, ok := q.Normalized()
	if !ok {
		t.Fatal("非零四元数应可归一化")
	}
	if math.Abs(n.Norm()-1) > 1e-15 {
		t.Errorf("归一化后模长 %.17g", n.Norm())
	}
	if _, ok := (Quat{}).Normalized(); ok {
		t.Error("零四元数不应可归一化")
	}
}

func TestExpRotationMatchesAxisAngle(t *testing.T) {
	axis := Vec3{X: 1, Y: 2, Z: 3}
	theta := 1.234
	v := ScaleVec(axis, theta/axis.Norm())
	q := ExpRotation(v)

	if math.Abs(q.Norm()-1) > 1e-15 {
		t.Errorf("指数映射结果模长 %.17g", q.Norm())
	}
	if got := RotationAngle(q); math.Abs(got-theta) > 1e-12 {
		t.Errorf("转角 %.12f，期望 %.12f", got, theta)
	}
	// 零向量 → 恒等
	if q := ExpRotation(Vec3{}); q != Identity() {
		t.Errorf("零旋转向量应映射到恒等，得到 %+v", q)
	}
}

func TestExpOfPureQuaternion(t *testing.T) {
	// 姿态四元数满足 q = exp((0, ωt)/2)，即 ExpRotation(v) = exp((0, v/2))。
	v := Vec3{X: 0.3, Y: -0.5, Z: 0.8}
	a := Exp(Pure(ScaleVec(v, 0.5)))
	b := ExpRotation(v)
	if d := maxDiff(a, b); d > 1e-14 {
		t.Errorf("exp((0,v/2)) 与 ExpRotation(v) 不一致：%.3e", d)
	}
}

// 运动学约定冒烟测试：dq/dt = 1/2 q⊗(0,ω) 沿 z 轴小步推进，
// 姿态应绕 z 轴转动（与欧拉角模块的 yaw 一致）。
func TestBodyRateKinematicsDirection(t *testing.T) {
	dt := 1e-6
	q := Identity()
	w := Vec3{Z: 1.0}
	for i := 0; i < 1000; i++ {
		dq := Mul(q, Pure(w)).Scale(0.5)
		q = Add(q, dq.Scale(dt))
		q, _ = q.Normalized()
	}
	// 1000 步 × 1e-6 s × 1 rad/s = 1e-3 rad 绕 z
	want := ExpRotation(Vec3{Z: 1e-3})
	if d := maxDiff(q, want); d > 1e-9 {
		t.Errorf("右乘运动学推进结果与绕 z 转 1e-3 rad 不符：%.3e", d)
	}
}

func TestGeodesicDistanceSignInvariant(t *testing.T) {
	q := ExpRotation(Vec3{X: 0.5, Y: 0.5, Z: 0.5})
	neg := q.Scale(-1)
	if d := GeodesicDistance(q, neg); d != 0 {
		t.Errorf("q 与 -q 是同一朝向，测地距离应为 0，得到 %.3e", d)
	}
}

// maxDiff 返回两个四元数逐分量差的最大绝对值（符号敏感，
// 用于“应精确相等”的断言；朝向等价请用 GeodesicDistance）。
func maxDiff(a, b Quat) float64 {
	d := math.Abs(a.W - b.W)
	if x := math.Abs(a.X - b.X); x > d {
		d = x
	}
	if x := math.Abs(a.Y - b.Y); x > d {
		d = x
	}
	if x := math.Abs(a.Z - b.Z); x > d {
		d = x
	}
	return d
}

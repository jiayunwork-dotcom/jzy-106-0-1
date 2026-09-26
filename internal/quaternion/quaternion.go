// Package quaternion 实现姿态积分所依赖的标量在前（scalar-first）
// Hamilton 四元数代数。本文件只做代数：乘法、归一化、共轭、指数映射等，
// 不包含任何时间积分逻辑（积分推进见 internal/integrator）。
package quaternion

import "math"

// Quat 是标量在前的四元数 q = W + X*i + Y*j + Z*k。
// 单位姿态四元数绕单位轴 a 转过角度 theta 时 W = cos(theta/2)。
type Quat struct {
	W, X, Y, Z float64
}

// Vec3 是机体系（body frame）下的三维向量。
type Vec3 struct {
	X, Y, Z float64
}

// Identity 返回乘法单位四元数（零姿态）。
func Identity() Quat { return Quat{W: 1} }

// IsFinite 判断四元数四个分量均非 NaN / Inf。
func (q Quat) IsFinite() bool {
	return math.IsInf(q.W, 0) == false && !math.IsNaN(q.W) &&
		math.IsInf(q.X, 0) == false && !math.IsNaN(q.X) &&
		math.IsInf(q.Y, 0) == false && !math.IsNaN(q.Y) &&
		math.IsInf(q.Z, 0) == false && !math.IsNaN(q.Z)
}

// NormSq 返回模长平方。
func (q Quat) NormSq() float64 {
	return q.W*q.W + q.X*q.X + q.Y*q.Y + q.Z*q.Z
}

// Norm 返回模长。
func (q Quat) Norm() float64 {
	return math.Sqrt(q.NormSq())
}

// Conj 返回共轭 q*（向量部分反号）。对单位四元数而言共轭即逆。
func (q Quat) Conj() Quat {
	return Quat{W: q.W, X: -q.X, Y: -q.Y, Z: -q.Z}
}

// Inv 返回四元数的逆 q^{-1} = q* / |q|^2。
func (q Quat) Inv() Quat {
	s := q.NormSq()
	if s == 0 {
		return Quat{}
	}
	c := q.Conj()
	return Quat{W: c.W / s, X: c.X / s, Y: c.Y / s, Z: c.Z / s}
}

// Normalized 返回 q/|q|；零四元数无法归一化，第二个返回值为 false。
func (q Quat) Normalized() (Quat, bool) {
	n := q.Norm()
	if n == 0 || math.IsNaN(n) || math.IsInf(n, 0) {
		return Quat{}, false
	}
	return Quat{W: q.W / n, X: q.X / n, Y: q.Y / n, Z: q.Z / n}, true
}

// Scale 返回标量乘法 s*q。
func (q Quat) Scale(s float64) Quat {
	return Quat{W: q.W * s, X: q.X * s, Y: q.Y * s, Z: q.Z * s}
}

// Add 返回逐分量加法 a+b。
func Add(a, b Quat) Quat {
	return Quat{W: a.W + b.W, X: a.X + b.X, Y: a.Y + b.Y, Z: a.Z + b.Z}
}

// Dot 返回四元数内积（逐分量乘积和）。
func Dot(a, b Quat) float64 {
	return a.W*b.W + a.X*b.X + a.Y*b.Y + a.Z*b.Z
}

// Mul 返回 Hamilton 乘积 a⊗b。
//
//	i*j = k, j*k = i, k*i = j（反交换）。
//	两个单位四元数相乘仍为单位四元数，对应两次旋转的复合。
func Mul(a, b Quat) Quat {
	return Quat{
		W: a.W*b.W - a.X*b.X - a.Y*b.Y - a.Z*b.Z,
		X: a.W*b.X + a.X*b.W + a.Y*b.Z - a.Z*b.Y,
		Y: a.W*b.Y - a.X*b.Z + a.Y*b.W + a.Z*b.X,
		Z: a.W*b.Z + a.X*b.Y - a.Y*b.X + a.Z*b.W,
	}
}

// Pure 把三维向量包成纯四元数 (0, v)。
func Pure(v Vec3) Quat {
	return Quat{X: v.X, Y: v.Y, Z: v.Z}
}

// AddVec 向量加法。
func AddVec(a, b Vec3) Vec3 {
	return Vec3{X: a.X + b.X, Y: a.Y + b.Y, Z: a.Z + b.Z}
}

// ScaleVec 向量数乘。
func ScaleVec(v Vec3, s float64) Vec3 {
	return Vec3{X: v.X * s, Y: v.Y * s, Z: v.Z * s}
}

// Norm 返回向量长度。
func (v Vec3) Norm() float64 {
	return math.Sqrt(v.X*v.X + v.Y*v.Y + v.Z*v.Z)
}

// Exp 返回一般四元数指数 exp(q)：对 q = (a, u)，
//
//	exp(q) = e^a (cos|u| + u/|u| sin|u|)
func Exp(q Quat) Quat {
	vn := math.Sqrt(q.X*q.X + q.Y*q.Y + q.Z*q.Z)
	ea := math.Exp(q.W)
	if vn < 1e-16 {
		return Quat{W: ea}
	}
	s := ea * math.Sin(vn) / vn
	return Quat{
		W: ea * math.Cos(vn),
		X: s * q.X,
		Y: s * q.Y,
		Z: s * q.Z,
	}
}

// ExpRotation 返回旋转向量 v 的指数映射，即绕 v 方向、转角 |v| 的单位四元数：
//
//	q = exp(v/2 对应的纯四元数)
//	  = (cos(|v|/2), v/|v| sin(|v|/2))
//
// 结果恒为单位四元数（v=0 时为恒等）。
func ExpRotation(v Vec3) Quat {
	theta := v.Norm()
	if theta < 1e-16 {
		return Identity()
	}
	h := 0.5 * theta
	s := math.Sin(h) / theta
	return Quat{
		W: math.Cos(h),
		X: v.X * s,
		Y: v.Y * s,
		Z: v.Z * s,
	}
}

// RotationAngle 返回单位四元数描述的旋转角，范围 [0, 2π)。
func RotationAngle(q Quat) float64 {
	return 2 * math.Atan2(math.Sqrt(q.X*q.X+q.Y*q.Y+q.Z*q.Z), q.W)
}

// GeodesicDistance 返回单位四元数在 SO(3) 上的测地距离（弧度），
// 自动消除 q 与 -q 代表同一朝向的符号歧义。
func GeodesicDistance(a, b Quat) float64 {
	d := math.Abs(Dot(a, b))
	if d > 1 {
		d = 1
	}
	return math.Acos(d)
}

package store

import (
	"errors"
	"testing"

	"attitude-service/internal/integrator"
	"attitude-service/internal/quaternion"
)

func steps(n int) []integrator.Sample {
	s := make([]integrator.Sample, n)
	for i := range s {
		s[i] = integrator.Sample{
			Omega: quaternion.Vec3{Z: 0.5},
			Dt:    0.01,
		}
	}
	return s
}

func TestSaveGetDelete(t *testing.T) {
	m := NewMemoryStore()
	if err := m.Save(Sequence{Name: "seq-a", Steps: steps(3)}); err != nil {
		t.Fatal(err)
	}
	got, err := m.Get("seq-a")
	if err != nil {
		t.Fatal(err)
	}
	if got.NumSamples != 3 || mathAbs(got.Duration-0.03) > 1e-12 {
		t.Errorf("元信息异常：%+v", got)
	}

	// 覆盖同名
	if err := m.Save(Sequence{Name: "seq-a", Steps: steps(5)}); err != nil {
		t.Fatal(err)
	}
	if got, _ := m.Get("seq-a"); got.NumSamples != 5 {
		t.Errorf("覆盖后采样数 = %d", got.NumSamples)
	}

	if err := m.Delete("seq-a"); err != nil {
		t.Fatal(err)
	}
	if _, err := m.Get("seq-a"); !errors.Is(err, ErrNotFound) {
		t.Errorf("删除后应 ErrNotFound，得到 %v", err)
	}
	if err := m.Delete("seq-a"); !errors.Is(err, ErrNotFound) {
		t.Errorf("删除不存在的序列应 ErrNotFound，得到 %v", err)
	}
}

func TestSaveDefensiveCopy(t *testing.T) {
	m := NewMemoryStore()
	src := steps(2)
	if err := m.Save(Sequence{Name: "s", Steps: src}); err != nil {
		t.Fatal(err)
	}
	// 外部修改入参不应影响存储
	src[0].Dt = 99
	got, _ := m.Get("s")
	if got.Steps[0].Dt != 0.01 {
		t.Errorf("存储被外部入参污染：%.4f", got.Steps[0].Dt)
	}
	// 修改取出的内容也不应影响存储
	got.Steps[1].Omega.Z = 7
	again, _ := m.Get("s")
	if again.Steps[1].Omega.Z != 0.5 {
		t.Errorf("存储被取出副本污染：%.4f", again.Steps[1].Omega.Z)
	}
}

func TestListSortedAndEmpty(t *testing.T) {
	m := NewMemoryStore()
	if list := m.List(); len(list) != 0 {
		t.Errorf("空存储列表应长度 0，得到 %d", len(list))
	}
	_ = m.Save(Sequence{Name: "zebra", Steps: steps(1)})
	_ = m.Save(Sequence{Name: "alpha", Steps: steps(2)})
	list := m.List()
	if len(list) != 2 || list[0].Name != "alpha" || list[1].Name != "zebra" {
		t.Errorf("列表未按名字排序：%+v", list)
	}
	if list[0].Steps != nil {
		t.Error("列表项不应携带完整采样数据")
	}
}

func TestSaveInvalid(t *testing.T) {
	m := NewMemoryStore()
	if err := m.Save(Sequence{Name: "", Steps: steps(1)}); err == nil {
		t.Error("空名字应报错")
	}
	if err := m.Save(Sequence{Name: "x", Steps: nil}); err == nil {
		t.Error("空序列应报错")
	}
}

func mathAbs(x float64) float64 {
	if x < 0 {
		return -x
	}
	return x
}

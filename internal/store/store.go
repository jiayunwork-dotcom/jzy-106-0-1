// Package store 提供命名角速度序列的存取。实现为进程内内存存储：
// 重启即丢失，调用方不得依赖持久性（需求允许）。
// 所有方法并发安全；存取的都是拷贝，外部修改不会渗进存储。
package store

import (
	"errors"
	"fmt"
	"sync"

	"attitude-service/internal/integrator"
)

// ErrNotFound 表示指定名字的序列不存在。
var ErrNotFound = errors.New("序列不存在")

// Sequence 是一条已命名的角速度序列。
type Sequence struct {
	Name       string
	Steps      []integrator.Sample
	NumSamples int
	Duration   float64
}

// Store 是命名序列存取接口。
type Store interface {
	Save(seq Sequence) error
	Get(name string) (Sequence, error)
	Delete(name string) error
	List() []Sequence
}

// MemoryStore 是并发安全的内存实现。
type MemoryStore struct {
	mu   sync.RWMutex
	data map[string]Sequence
}

// NewMemoryStore 创建空的内存存储。
func NewMemoryStore() *MemoryStore {
	return &MemoryStore{data: make(map[string]Sequence)}
}

// Save 保存（或覆盖）一条序列；Name 与 Steps 的合法性由调用方
// （internal/validation）保证，这里只做防御性检查并深拷贝。
func (m *MemoryStore) Save(seq Sequence) error {
	if seq.Name == "" {
		return errors.New("序列名称不能为空")
	}
	if len(seq.Steps) == 0 {
		return errors.New("序列为空：至少需要一个采样")
	}
	m.mu.Lock()
	defer m.mu.Unlock()
	cp := cloneSteps(seq.Steps)
	dur := 0.0
	for _, s := range cp {
		dur += s.Dt
	}
	m.data[seq.Name] = Sequence{
		Name:       seq.Name,
		Steps:      cp,
		NumSamples: len(cp),
		Duration:   dur,
	}
	return nil
}

// Get 取出一条序列（深拷贝）。
func (m *MemoryStore) Get(name string) (Sequence, error) {
	m.mu.RLock()
	defer m.mu.RUnlock()
	seq, ok := m.data[name]
	if !ok {
		return Sequence{}, fmt.Errorf("%w：%q", ErrNotFound, name)
	}
	seq.Steps = cloneSteps(seq.Steps)
	return seq, nil
}

// Delete 删除一条序列。
func (m *MemoryStore) Delete(name string) error {
	m.mu.Lock()
	defer m.mu.Unlock()
	if _, ok := m.data[name]; !ok {
		return fmt.Errorf("%w：%q", ErrNotFound, name)
	}
	delete(m.data, name)
	return nil
}

// List 返回全部序列的元信息（按名字排序）。
func (m *MemoryStore) List() []Sequence {
	m.mu.RLock()
	defer m.mu.RUnlock()
	names := make([]string, 0, len(m.data))
	for n := range m.data {
		names = append(names, n)
	}
	sortStrings(names)
	out := make([]Sequence, 0, len(names))
	for _, n := range names {
		s := m.data[n]
		s.Steps = nil // 列表只给元信息
		out = append(out, s)
	}
	return out
}

func cloneSteps(in []integrator.Sample) []integrator.Sample {
	out := make([]integrator.Sample, len(in))
	copy(out, in)
	return out
}

// sortStrings 避免引入 sort 包只为这一处（Go 1.22 用 sort.Strings 亦可）。
func sortStrings(s []string) {
	for i := 1; i < len(s); i++ {
		for j := i; j > 0 && s[j-1] > s[j]; j-- {
			s[j-1], s[j] = s[j], s[j-1]
		}
	}
}

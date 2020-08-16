package robot

import (
	"testing"

	"github.com/stretchr/testify/assert"
)

func TestForwardAndInverse(t *testing.T) {
	r := &Robot{
		BaseRadius:          75.0,
		BicepLength:         100.0,
		ForearmLength:       300.0,
		EndEffectorRadius:   24.0,
		BaseToFloorDistance: 400.0,
	}

	// Test with arms at 5, 10 and 15 degrees
	x, y, z, err := r.Forward(5.0, 10.0, 15.0)
	assert.Nil(t, err)
	assert.InDelta(t, x, 13.131, 0.001)
	assert.InDelta(t, y, -22.505, 0.001)
	assert.InDelta(t, z, -294.011, 0.001)

	t1, t2, t3, err := r.Inverse(x, y, z)
	assert.Nil(t, err)
	assert.InDelta(t, t1, 5.0, 0.001)
	assert.InDelta(t, t2, 10.0, 0.001)
	assert.InDelta(t, t3, 15.0, 0.001)

	// Test with arms flat
	x, y, z, err = r.Forward(0.0, 0.0, 0.0)
	assert.Nil(t, err)
	assert.InDelta(t, x, 0.0, 0.01)
	assert.InDelta(t, y, 0.0, 0.01)
	assert.InDelta(t, z, -277.198, 0.01)

	t1, t2, t3, err = r.Inverse(x, y, z)
	assert.Nil(t, err)
	assert.InDelta(t, t1, 0.0, 0.001)
	assert.InDelta(t, t2, 0.0, 0.001)
	assert.InDelta(t, t3, 0.0, 0.001)
}

<script setup lang="ts">
import { computed } from 'vue'
import { useNTInteger } from '../composables/useNetworkTables'
import { useCalibrationStore } from '../stores/calibration'

const store = useCalibrationStore()

const currentRow = useNTInteger('/TurretCalibration/Grid/CurrentRow', 0)
const currentCol = useNTInteger('/TurretCalibration/Grid/CurrentCol', 0)

const gridCells = computed(() => {
  const cells = []
  for (let row = 0; row < store.gridRows; row++) {
    for (let col = 0; col < store.gridCols; col++) {
      const hasPoint = store.hasPointAt(row, col)
      const isCurrent = row === currentRow.value && col === currentCol.value
      cells.push({ row, col, hasPoint, isCurrent })
    }
  }
  return cells
})
</script>

<template>
  <div class="calibration-grid">
    <h3>Calibration Grid</h3>

    <div class="grid-legend">
      <span class="legend-item">
        <span class="cell-sample current"></span>
        Current Position
      </span>
      <span class="legend-item">
        <span class="cell-sample filled"></span>
        Calibrated
      </span>
      <span class="legend-item">
        <span class="cell-sample empty"></span>
        Empty
      </span>
    </div>

    <div class="grid-container">
      <div
        v-for="cell in gridCells"
        :key="`${cell.row}-${cell.col}`"
        class="grid-cell"
        :class="{
          'has-point': cell.hasPoint,
          'is-current': cell.isCurrent
        }"
        :title="`[${cell.row}, ${cell.col}]`"
      ></div>
    </div>

    <div class="grid-labels">
      <span>Y=0</span>
      <span class="spacer"></span>
      <span>Y=max</span>
    </div>
  </div>
</template>

<style scoped>
.calibration-grid {
  background: #252540;
  border-radius: 8px;
  padding: 16px;
}

h3 {
  margin: 0 0 12px 0;
  font-size: 14px;
  color: #888;
  text-transform: uppercase;
  letter-spacing: 1px;
}

.grid-legend {
  display: flex;
  gap: 16px;
  margin-bottom: 12px;
  font-size: 12px;
  color: #888;
}

.legend-item {
  display: flex;
  align-items: center;
  gap: 6px;
}

.cell-sample {
  width: 12px;
  height: 12px;
  border-radius: 2px;
}

.cell-sample.current {
  background: #ff9800;
  box-shadow: 0 0 6px #ff9800;
}

.cell-sample.filled {
  background: #4caf50;
}

.cell-sample.empty {
  background: #333;
}

.grid-container {
  display: grid;
  grid-template-columns: repeat(17, 1fr);
  gap: 2px;
  background: #1a1a2e;
  padding: 8px;
  border-radius: 4px;
}

.grid-cell {
  aspect-ratio: 1;
  background: #333;
  border-radius: 2px;
  transition: all 0.2s;
}

.grid-cell.has-point {
  background: #4caf50;
}

.grid-cell.is-current {
  background: #ff9800;
  box-shadow: 0 0 6px #ff9800;
}

.grid-cell.has-point.is-current {
  background: #ff9800;
}

.grid-labels {
  display: flex;
  justify-content: space-between;
  margin-top: 8px;
  font-size: 11px;
  color: #666;
}

.spacer {
  flex: 1;
}
</style>

<script setup lang="ts">
import { computed, watch } from 'vue'
import { useNTInteger } from '../composables/useNetworkTables'
import { useCalibrationStore } from '../stores/calibration'
import { useFieldCoordinates } from '../composables/useFieldCoordinates'

const store = useCalibrationStore()
const { gridOverlayStyle, fieldImagePath } = useFieldCoordinates()

const currentRow = useNTInteger('/TurretCalibration/Grid/CurrentRow', 0)
const currentCol = useNTInteger('/TurretCalibration/Grid/CurrentCol', 0)

// Read grid dimensions from NetworkTables (updates store when changed)
const ntGridRows = useNTInteger('/TurretCalibration/Grid/Rows', 13)
const ntGridCols = useNTInteger('/TurretCalibration/Grid/Cols', 29)

// Update store when NT values change
watch([ntGridRows, ntGridCols], ([rows, cols]) => {
  if (rows > 0 && cols > 0) {
    store.setGridDimensions(rows, cols)
  }
}, { immediate: true })

const gridCells = computed(() => {
  const cells = []
  // Iterate rows in reverse order so Y increases from bottom to top (matching field coordinates)
  for (let row = store.gridRows - 1; row >= 0; row--) {
    for (let col = 0; col < store.gridCols; col++) {
      const hasPoint = store.hasPointAt(row, col)
      const isCurrent = row === currentRow.value && col === currentCol.value
      cells.push({ row, col, hasPoint, isCurrent })
    }
  }
  return cells
})

// Dynamic grid template columns based on store.gridCols
const gridStyle = computed(() => ({
  gridTemplateColumns: `repeat(${store.gridCols}, 1fr)`,
  gridTemplateRows: `repeat(${store.gridRows}, 1fr)`,
}))
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

    <div class="field-container">
      <img :src="fieldImagePath" alt="FRC Field" class="field-image" />
      <div class="grid-overlay" :style="gridOverlayStyle">
        <div class="overlay-grid" :style="gridStyle">
          <div
            v-for="cell in gridCells"
            :key="`${cell.row}-${cell.col}`"
            class="grid-cell"
            :class="{
              'has-point': cell.hasPoint,
              'is-current': cell.isCurrent
            }"
            :title="`[${cell.row}, ${cell.col}] - X: ${(1.0 + cell.col * 0.5).toFixed(1)}m, Y: ${(1.0 + cell.row * 0.5).toFixed(1)}m`"
          ></div>
        </div>
      </div>
    </div>

    <div class="axis-info">
      <span class="axis-label">X: 1.0m → 15.5m</span>
      <span class="axis-label">Y: 1.0m → 7.0m</span>
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
  background: rgba(255, 152, 0, 0.8);
  box-shadow: 0 0 6px #ff9800;
}

.cell-sample.filled {
  background: rgba(76, 175, 80, 0.55);
}

.cell-sample.empty {
  background: rgba(51, 51, 51, 0.4);
  border: 1px solid rgba(255, 255, 255, 0.1);
}

.field-container {
  position: relative;
  width: 100%;
  border-radius: 4px;
  overflow: hidden;
}

.field-image {
  display: block;
  width: 100%;
  height: auto;
}

.grid-overlay {
  position: absolute;
  pointer-events: none;
}

.overlay-grid {
  display: grid;
  width: 100%;
  height: 100%;
  gap: 1px;
}

.grid-cell {
  background: rgba(51, 51, 51, 0.4);
  border: 1px solid rgba(255, 255, 255, 0.1);
  border-radius: 1px;
  transition: all 0.2s;
  pointer-events: auto;
}

.grid-cell.has-point {
  background: rgba(76, 175, 80, 0.55);
  border-color: rgba(76, 175, 80, 0.7);
}

.grid-cell.is-current {
  background: rgba(255, 152, 0, 0.8);
  border-color: rgba(255, 152, 0, 1);
  box-shadow: 0 0 8px rgba(255, 152, 0, 0.8);
  animation: pulse 1.5s ease-in-out infinite;
}

.grid-cell.has-point.is-current {
  background: rgba(255, 152, 0, 0.8);
}

@keyframes pulse {
  0%, 100% {
    box-shadow: 0 0 8px rgba(255, 152, 0, 0.8);
  }
  50% {
    box-shadow: 0 0 16px rgba(255, 152, 0, 1);
  }
}

.axis-info {
  display: flex;
  justify-content: space-between;
  margin-top: 8px;
  font-size: 11px;
  color: #888;
}

.axis-label {
  font-family: 'Consolas', monospace;
  color: #aaa;
}
</style>

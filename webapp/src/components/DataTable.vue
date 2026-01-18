<script setup lang="ts">
import { computed } from 'vue'
import { useCalibrationStore } from '../stores/calibration'

const store = useCalibrationStore()

const sortedPoints = computed(() =>
  [...store.points].sort((a, b) => {
    if (a.gridRow !== b.gridRow) return a.gridRow - b.gridRow
    return a.gridCol - b.gridCol
  })
)

const removePoint = (row: number, col: number) => {
  store.removePoint(row, col)
}
</script>

<template>
  <div class="data-table">
    <h3>Calibration Points</h3>

    <div v-if="store.points.length === 0" class="empty-state">
      No calibration points saved yet.
      <br />
      Position the robot and click "Save Point" to begin.
    </div>

    <div v-else class="table-container">
      <table>
        <thead>
          <tr>
            <th>Grid</th>
            <th>Distance</th>
            <th>Hood</th>
            <th>RPM</th>
            <th></th>
          </tr>
        </thead>
        <tbody>
          <tr v-for="point in sortedPoints" :key="`${point.gridRow}-${point.gridCol}`">
            <td class="cell-grid">[{{ point.gridRow }}, {{ point.gridCol }}]</td>
            <td class="cell-number">{{ point.distanceToTarget.toFixed(2) }}m</td>
            <td class="cell-number">{{ point.hoodAngleDegrees.toFixed(1) }}°</td>
            <td class="cell-number">{{ point.flywheelRPM.toFixed(0) }}</td>
            <td class="cell-action">
              <button @click="removePoint(point.gridRow, point.gridCol)" class="delete-btn">
                ×
              </button>
            </td>
          </tr>
        </tbody>
      </table>
    </div>
  </div>
</template>

<style scoped>
.data-table {
  background: #252540;
  border-radius: 8px;
  padding: 16px;
  max-height: 400px;
  display: flex;
  flex-direction: column;
}

h3 {
  margin: 0 0 12px 0;
  font-size: 14px;
  color: #888;
  text-transform: uppercase;
  letter-spacing: 1px;
}

.empty-state {
  text-align: center;
  padding: 32px;
  color: #666;
  font-size: 14px;
  line-height: 1.6;
}

.table-container {
  overflow-y: auto;
  flex: 1;
}

table {
  width: 100%;
  border-collapse: collapse;
  font-size: 13px;
}

th {
  text-align: left;
  padding: 8px;
  background: #1a1a2e;
  color: #888;
  font-weight: 500;
  position: sticky;
  top: 0;
}

td {
  padding: 8px;
  border-bottom: 1px solid #333;
}

tr:hover td {
  background: rgba(63, 81, 181, 0.1);
}

.cell-grid {
  font-family: 'Consolas', monospace;
  color: #7986cb;
}

.cell-number {
  font-family: 'Consolas', monospace;
  text-align: right;
}

.cell-action {
  text-align: center;
  width: 40px;
}

.delete-btn {
  background: transparent;
  border: none;
  color: #666;
  font-size: 18px;
  cursor: pointer;
  padding: 2px 8px;
  border-radius: 4px;
}

.delete-btn:hover {
  background: rgba(244, 67, 54, 0.2);
  color: #f44336;
}
</style>

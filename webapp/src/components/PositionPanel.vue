<script setup lang="ts">
import { computed } from 'vue'
import { useNTDouble, useNTInteger } from '../composables/useNetworkTables'
import { TOPICS } from '../constants/topics'

const posX = useNTDouble(TOPICS.POSITION_X, 0)
const posY = useNTDouble(TOPICS.POSITION_Y, 0)
const distance = useNTDouble(TOPICS.DISTANCE, 0)
const gridRow = useNTInteger(TOPICS.GRID.CURRENT_ROW, 0)
const gridCol = useNTInteger(TOPICS.GRID.CURRENT_COL, 0)
const nextTargetX = useNTDouble(TOPICS.GRID.NEXT_TARGET_X, 0)
const nextTargetY = useNTDouble(TOPICS.GRID.NEXT_TARGET_Y, 0)

const formattedX = computed(() => posX.value.toFixed(2))
const formattedY = computed(() => posY.value.toFixed(2))
const formattedDistance = computed(() => distance.value.toFixed(2))
</script>

<template>
  <div class="position-panel">
    <h3>Robot Position</h3>

    <div class="position-grid">
      <div class="position-item">
        <span class="label">X</span>
        <span class="value">{{ formattedX }} m</span>
      </div>
      <div class="position-item">
        <span class="label">Y</span>
        <span class="value">{{ formattedY }} m</span>
      </div>
      <div class="position-item highlight">
        <span class="label">Distance</span>
        <span class="value">{{ formattedDistance }} m</span>
      </div>
    </div>

    <div class="grid-info">
      <div class="grid-current">
        <span class="label">Grid Cell</span>
        <span class="value">[{{ gridRow }}, {{ gridCol }}]</span>
      </div>
      <div class="grid-next">
        <span class="label">Next Target</span>
        <span class="value">({{ nextTargetX.toFixed(1) }}, {{ nextTargetY.toFixed(1) }})</span>
      </div>
    </div>
  </div>
</template>

<style scoped>
.position-panel {
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

.position-grid {
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  gap: 12px;
  margin-bottom: 16px;
}

.position-item {
  background: #1a1a2e;
  border-radius: 6px;
  padding: 12px;
  text-align: center;
}

.position-item.highlight {
  background: #2a2a4a;
  border: 1px solid #3f51b5;
}

.label {
  display: block;
  font-size: 12px;
  color: #888;
  margin-bottom: 4px;
}

.value {
  display: block;
  font-size: 18px;
  font-weight: 600;
  font-family: 'Consolas', monospace;
}

.grid-info {
  display: flex;
  gap: 16px;
}

.grid-current,
.grid-next {
  flex: 1;
  background: #1a1a2e;
  border-radius: 6px;
  padding: 10px;
  text-align: center;
}

.grid-info .value {
  font-size: 14px;
}
</style>

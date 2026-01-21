<script setup lang="ts">
import { ref, watch } from 'vue'
import { useDebounceFn } from '@vueuse/core'
import { useNTDouble, useNTBoolean, publishDouble } from '../composables/useNetworkTables'
import { TOPICS } from '../constants/topics'

// Read actual values from robot
const actualHoodAngle = useNTDouble(TOPICS.ACTUAL_HOOD_ANGLE, 25)
const actualFlywheelRPM = useNTDouble(TOPICS.ACTUAL_FLYWHEEL_RPM, 3000)
const hoodAtPosition = useNTBoolean(TOPICS.HOOD_AT_POSITION, false)
const flywheelAtSetpoint = useNTBoolean(TOPICS.FLYWHEEL_AT_SETPOINT, false)

// Read constants from robot (with fallback defaults)
const minHoodAngle = useNTDouble(TOPICS.CONSTANTS.MIN_HOOD_ANGLE, 0)
const maxHoodAngle = useNTDouble(TOPICS.CONSTANTS.MAX_HOOD_ANGLE, 90)
const minFlywheelRPM = useNTDouble(TOPICS.CONSTANTS.MIN_FLYWHEEL_RPM, 1000)
const maxFlywheelRPM = useNTDouble(TOPICS.CONSTANTS.MAX_FLYWHEEL_RPM, 6000)

// Input values (local state, published to NT)
const inputHoodAngle = ref(25)
const inputFlywheelRPM = ref(3000)

// Track whether constants have been received from robot
const constantsReceived = ref(false)

// Debounced publish functions (100ms debounce to reduce network traffic)
const debouncedPublishHoodAngle = useDebounceFn((value: number) => {
  publishDouble(TOPICS.INPUT_HOOD_ANGLE, value)
}, 100)

const debouncedPublishFlywheelRPM = useDebounceFn((value: number) => {
  publishDouble(TOPICS.INPUT_FLYWHEEL_RPM, value)
}, 100)

// Publish changes to NetworkTables with debouncing (only after constants received)
watch(inputHoodAngle, (value) => {
  if (constantsReceived.value) {
    debouncedPublishHoodAngle(value)
  }
})

watch(inputFlywheelRPM, (value) => {
  if (constantsReceived.value) {
    debouncedPublishFlywheelRPM(value)
  }
})

// Unified watcher for all constants - waits for all values before publishing
// This prevents race condition where separate watchers could fire out of order
// and publish unclamped values
watch(
  [minHoodAngle, maxHoodAngle, minFlywheelRPM, maxFlywheelRPM],
  ([newMinHood, newMaxHood, newMinRPM, newMaxRPM]) => {
    // Clamp hood angle to valid range
    if (inputHoodAngle.value < newMinHood) {
      inputHoodAngle.value = newMinHood
    } else if (inputHoodAngle.value > newMaxHood) {
      inputHoodAngle.value = newMaxHood
    }

    // Clamp flywheel RPM to valid range
    if (inputFlywheelRPM.value < newMinRPM) {
      inputFlywheelRPM.value = newMinRPM
    } else if (inputFlywheelRPM.value > newMaxRPM) {
      inputFlywheelRPM.value = newMaxRPM
    }

    // On first receipt of constants, publish clamped initial values
    if (!constantsReceived.value) {
      constantsReceived.value = true
      // Publish clamped initial values immediately (no debounce for initialization)
      publishDouble(TOPICS.INPUT_HOOD_ANGLE, inputHoodAngle.value)
      publishDouble(TOPICS.INPUT_FLYWHEEL_RPM, inputFlywheelRPM.value)
    }
  }
)

// Blur handlers to clamp values when user finishes typing
// HTML5 min/max attributes don't prevent typing invalid values
const clampHoodAngle = () => {
  if (inputHoodAngle.value < minHoodAngle.value) {
    inputHoodAngle.value = minHoodAngle.value
  } else if (inputHoodAngle.value > maxHoodAngle.value) {
    inputHoodAngle.value = maxHoodAngle.value
  }
}

const clampFlywheelRPM = () => {
  if (inputFlywheelRPM.value < minFlywheelRPM.value) {
    inputFlywheelRPM.value = minFlywheelRPM.value
  } else if (inputFlywheelRPM.value > maxFlywheelRPM.value) {
    inputFlywheelRPM.value = maxFlywheelRPM.value
  }
}
</script>

<template>
  <div class="control-panel">
    <h3>Controls</h3>

    <div class="control-group">
      <div class="control-header">
        <label>Hood Angle</label>
        <span class="setpoint-indicator" :class="{ reached: hoodAtPosition }">
          {{ hoodAtPosition ? 'At Position' : 'Moving...' }}
        </span>
      </div>

      <div class="slider-row">
        <input
          type="range"
          v-model.number="inputHoodAngle"
          :min="minHoodAngle"
          :max="maxHoodAngle"
          step="0.5"
        />
        <input
          type="number"
          v-model.number="inputHoodAngle"
          :min="minHoodAngle"
          :max="maxHoodAngle"
          step="0.5"
          class="value-input"
          @blur="clampHoodAngle"
        />
        <span class="unit">deg</span>
      </div>

      <div class="actual-value">
        Actual: {{ actualHoodAngle.toFixed(1) }}°
      </div>
    </div>

    <div class="control-group">
      <div class="control-header">
        <label>Flywheel RPM</label>
        <span class="setpoint-indicator" :class="{ reached: flywheelAtSetpoint }">
          {{ flywheelAtSetpoint ? 'At Setpoint' : 'Spinning Up...' }}
        </span>
      </div>

      <div class="slider-row">
        <input
          type="range"
          v-model.number="inputFlywheelRPM"
          :min="minFlywheelRPM"
          :max="maxFlywheelRPM"
          step="50"
        />
        <input
          type="number"
          v-model.number="inputFlywheelRPM"
          :min="minFlywheelRPM"
          :max="maxFlywheelRPM"
          step="50"
          class="value-input"
          @blur="clampFlywheelRPM"
        />
        <span class="unit">RPM</span>
      </div>

      <div class="actual-value">
        Actual: {{ actualFlywheelRPM.toFixed(0) }} RPM
      </div>
    </div>
  </div>
</template>

<style scoped>
.control-panel {
  background: #252540;
  border-radius: 8px;
  padding: 16px;
}

h3 {
  margin: 0 0 16px 0;
  font-size: 14px;
  color: #888;
  text-transform: uppercase;
  letter-spacing: 1px;
}

.control-group {
  margin-bottom: 20px;
}

.control-group:last-child {
  margin-bottom: 0;
}

.control-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 8px;
}

.control-header label {
  font-weight: 500;
  font-size: 14px;
}

.setpoint-indicator {
  font-size: 12px;
  padding: 2px 8px;
  border-radius: 10px;
  background: #444;
  color: #aaa;
}

.setpoint-indicator.reached {
  background: rgba(76, 175, 80, 0.2);
  color: #4caf50;
}

.slider-row {
  display: flex;
  align-items: center;
  gap: 12px;
}

input[type="range"] {
  flex: 1;
  height: 6px;
  border-radius: 3px;
  background: #1a1a2e;
  appearance: none;
  outline: none;
}

input[type="range"]::-webkit-slider-thumb {
  appearance: none;
  width: 18px;
  height: 18px;
  border-radius: 50%;
  background: #3f51b5;
  cursor: pointer;
}

input[type="range"]::-webkit-slider-thumb:hover {
  background: #5c6bc0;
}

.value-input {
  width: 70px;
  padding: 6px 8px;
  border: 1px solid #444;
  border-radius: 4px;
  background: #1a1a2e;
  color: #eee;
  font-size: 14px;
  text-align: right;
}

.value-input:focus {
  outline: none;
  border-color: #3f51b5;
}

.unit {
  font-size: 12px;
  color: #888;
  width: 30px;
}

.actual-value {
  margin-top: 6px;
  font-size: 12px;
  color: #888;
  font-family: 'Consolas', monospace;
}
</style>

<script setup lang="ts">
import { ref, watch } from 'vue'
import { useDebounceFn } from '@vueuse/core'
import { useNTDouble, useNTBoolean, publishDouble } from '../composables/useNetworkTables'
import { TOPICS } from '../constants/topics'

const calibrationEnabled = useNTBoolean(TOPICS.ENABLED, false)
const ntInputHoodAngle = useNTDouble(TOPICS.INPUT_HOOD_ANGLE, 25)
const ntInputFlywheelRPM = useNTDouble(TOPICS.INPUT_FLYWHEEL_RPM, 3000)
const robotObservedInputHoodAngle = useNTDouble(TOPICS.OBSERVED_INPUT_HOOD_ANGLE, 25)
const robotObservedInputFlywheelRPM = useNTDouble(TOPICS.OBSERVED_INPUT_FLYWHEEL_RPM, 3000)

// Read actual values from robot
const actualHoodAngle = useNTDouble(TOPICS.ACTUAL_HOOD_ANGLE, 25)
const actualFlywheelRPM = useNTDouble(TOPICS.ACTUAL_FLYWHEEL_RPM, 3000)
const hoodAtPosition = useNTBoolean(TOPICS.HOOD_AT_POSITION, false)
const flywheelAtSetpoint = useNTBoolean(TOPICS.FLYWHEEL_AT_SETPOINT, false)

// Read constants from robot (with fallback defaults)
const minHoodAngle = useNTDouble(TOPICS.CONSTANTS.MIN_HOOD_ANGLE, 0)
const maxHoodAngle = useNTDouble(TOPICS.CONSTANTS.MAX_HOOD_ANGLE, 25)
const minFlywheelRPM = useNTDouble(TOPICS.CONSTANTS.MIN_FLYWHEEL_RPM, 500)
const maxFlywheelRPM = useNTDouble(TOPICS.CONSTANTS.MAX_FLYWHEEL_RPM, 4700)

// Input values (local state, published to NT)
const inputHoodAngle = ref(25)
const inputFlywheelRPM = ref(3000)

// Track whether constants have been received from robot
const constantsReceived = ref(false)
const publishAttemptIds = new Map<string, number>()

const nextPublishAttemptId = (topic: string): number => {
  const nextId = (publishAttemptIds.get(topic) ?? 0) + 1
  publishAttemptIds.set(topic, nextId)
  return nextId
}

const schedulePublishVerification = (
  topic: string,
  label: string,
  attemptId: number,
  requestedValue: number,
  echoedValue: { value: number },
  robotReadbackValue: { value: number },
  tolerance: number
) => {
  const verify = (checkRobotReadback: boolean) => {
    if (publishAttemptIds.get(topic) !== attemptId) {
      return
    }

    if (Math.abs(echoedValue.value - requestedValue) > tolerance) {
      console.warn(
        `NetworkTables publish mismatch for ${label}: requested=${requestedValue}, echoed=${echoedValue.value}.`
      )
    }

    if (checkRobotReadback && calibrationEnabled.value &&
        Math.abs(robotReadbackValue.value - requestedValue) > tolerance) {
      console.warn(
        `Robot readback mismatch for ${label}: requested=${requestedValue}, robot=${robotReadbackValue.value}.`
      )
    }
  }

  window.setTimeout(() => verify(false), 250)
  window.setTimeout(() => verify(true), 1000)
}

const publishWithVerification = (
  topic: string,
  label: string,
  value: number,
  echoedValue: { value: number },
  robotReadbackValue: { value: number },
  tolerance: number
) => {
  const attemptId = nextPublishAttemptId(topic)
  console.log(`NetworkTables publish ${label}: ${value}`)
  void publishDouble(topic, value)
    .then(() => {
      schedulePublishVerification(
        topic,
        label,
        attemptId,
        value,
        echoedValue,
        robotReadbackValue,
        tolerance
      )
    })
    .catch((error) => {
      console.error(`NetworkTables publish failed for ${label}`, error)
    })
}

// Debounced publish functions (100ms debounce to reduce network traffic)
const debouncedPublishHoodAngle = useDebounceFn((value: number) => {
  publishWithVerification(
    TOPICS.INPUT_HOOD_ANGLE,
    'hood angle',
    value,
    ntInputHoodAngle,
    robotObservedInputHoodAngle,
    0.05
  )
}, 100)

const debouncedPublishFlywheelRPM = useDebounceFn((value: number) => {
  publishWithVerification(
    TOPICS.INPUT_FLYWHEEL_RPM,
    'flywheel rpm',
    value,
    ntInputFlywheelRPM,
    robotObservedInputFlywheelRPM,
    1.0
  )
}, 100)

// Publish changes to NetworkTables with debouncing
watch(inputHoodAngle, (value) => {
  debouncedPublishHoodAngle(value)
})

watch(inputFlywheelRPM, (value) => {
  debouncedPublishFlywheelRPM(value)
})

watch(ntInputHoodAngle, (value) => {
  console.log(`NetworkTables echo hood angle: ${value}`)
})

watch(ntInputFlywheelRPM, (value) => {
  console.log(`NetworkTables echo flywheel rpm: ${value}`)
})

// Clamp inputs when constants change and publish initial values on startup
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

    // Publish initial values so robot receives them immediately
    if (!constantsReceived.value) {
      constantsReceived.value = true
      publishWithVerification(
        TOPICS.INPUT_HOOD_ANGLE,
        'hood angle',
        inputHoodAngle.value,
        ntInputHoodAngle,
        robotObservedInputHoodAngle,
        0.05
      )
      publishWithVerification(
        TOPICS.INPUT_FLYWHEEL_RPM,
        'flywheel rpm',
        inputFlywheelRPM.value,
        ntInputFlywheelRPM,
        robotObservedInputFlywheelRPM,
        1.0
      )
    }
  },
  { immediate: true }
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
  <div class="control-panel" :class="{ inactive: !calibrationEnabled }">
    <h3>Controls</h3>

    <div v-if="!calibrationEnabled" class="inactive-warning">
      Calibration inactive. Put the robot in test mode and hold the driver right trigger to apply web app setpoints.
    </div>

    <fieldset class="controls-fieldset" :disabled="!calibrationEnabled">

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
      <div class="nt-echo">
        NT Echo: {{ ntInputHoodAngle.toFixed(2) }} deg
      </div>
      <div class="nt-echo robot-echo">
        Robot Readback: {{ robotObservedInputHoodAngle.toFixed(2) }} deg
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
      <div class="nt-echo">
        NT Echo: {{ ntInputFlywheelRPM.toFixed(1) }} RPM
      </div>
      <div class="nt-echo robot-echo">
        Robot Readback: {{ robotObservedInputFlywheelRPM.toFixed(1) }} RPM
      </div>
    </div>

    </fieldset>
  </div>
</template>

<style scoped>
.control-panel {
  background: #252540;
  border-radius: 8px;
  padding: 16px;
}

.control-panel.inactive {
  opacity: 0.78;
}

h3 {
  margin: 0 0 16px 0;
  font-size: 14px;
  color: #888;
  text-transform: uppercase;
  letter-spacing: 1px;
}

.inactive-warning {
  margin-bottom: 16px;
  padding: 10px 12px;
  border-radius: 6px;
  background: rgba(255, 152, 0, 0.12);
  border: 1px solid rgba(255, 152, 0, 0.35);
  color: #ffb74d;
  font-size: 12px;
  line-height: 1.4;
}

.controls-fieldset {
  margin: 0;
  padding: 0;
  border: 0;
  min-width: 0;
}

.controls-fieldset:disabled {
  cursor: not-allowed;
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

.controls-fieldset:disabled input[type="range"] {
  cursor: not-allowed;
  opacity: 0.65;
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

.controls-fieldset:disabled .value-input {
  opacity: 0.65;
  cursor: not-allowed;
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

.nt-echo {
  margin-top: 4px;
  font-size: 11px;
  color: #6fb3ff;
  font-family: 'Consolas', monospace;
}

.robot-echo {
  color: #9be37a;
}
</style>

<script setup lang="ts">
import { ref, computed, onMounted, onUnmounted } from 'vue'
import { useNTBoolean, useNTString, useNTDouble, useNTConnection, reconnect } from '../composables/useNetworkTables'
import { useCalibrationStore } from '../stores/calibration'
import { TOPICS } from '../constants/topics'

const store = useCalibrationStore()
const { connected, serverAddress, reconnecting } = useNTConnection()
const enabled = useNTBoolean(TOPICS.ENABLED, false)
const status = useNTString(TOPICS.STATUS, 'Not Connected')
const error = useNTString(TOPICS.ERROR, '')
const lastUpdateTimestamp = useNTDouble(TOPICS.DATA.LAST_UPDATE_TIMESTAMP, 0)

// Data quality indicator based on calibration coverage
const dataQuality = computed(() => {
  const percentage = store.completionPercentage
  const pointCount = store.pointCount
  if (pointCount === 0) {
    return { text: 'No Data', level: 'none', color: '#666' }
  }
  if (percentage >= 80) {
    return { text: 'Excellent', level: 'excellent', color: '#4caf50' }
  }
  if (percentage >= 50) {
    return { text: 'Good', level: 'good', color: '#8bc34a' }
  }
  if (percentage >= 25) {
    return { text: 'Partial', level: 'partial', color: '#ff9800' }
  }
  return { text: 'Low', level: 'low', color: '#f44336' }
})

// Track current time for freshness calculation
const currentTime = ref(Date.now() / 1000)
let timeInterval: ReturnType<typeof setInterval> | null = null

onMounted(() => {
  timeInterval = setInterval(() => {
    currentTime.value = Date.now() / 1000
  }, 1000)
})

onUnmounted(() => {
  if (timeInterval) {
    clearInterval(timeInterval)
  }
})

// Calculate data freshness
// Thresholds: 5s = live, 10s = warning, 30s = stale
const dataFreshness = computed(() => {
  if (lastUpdateTimestamp.value === 0) {
    return { text: 'No data', stale: true, warning: false }
  }
  const secondsAgo = Math.floor(currentTime.value - lastUpdateTimestamp.value)
  // Use Math.abs to handle minor clock skew (robot clock slightly ahead)
  const absSecondsAgo = Math.abs(secondsAgo)
  if (absSecondsAgo > 30) {
    return { text: 'Stale', stale: true, warning: false }
  }
  if (absSecondsAgo > 10) {
    return { text: `${absSecondsAgo}s ago`, stale: false, warning: true }
  }
  if (absSecondsAgo <= 5) {
    return { text: 'Live', stale: false, warning: false }
  }
  // Show positive value even if clock skew causes negative
  return { text: `${absSecondsAgo}s ago`, stale: false, warning: false }
})

const editingAddress = ref(false)
const addressInput = ref(serverAddress.value)

const startEditingAddress = () => {
  addressInput.value = serverAddress.value
  editingAddress.value = true
}

const cancelEditingAddress = () => {
  editingAddress.value = false
  addressInput.value = serverAddress.value
}

const submitAddress = () => {
  const newAddress = addressInput.value.trim()
  if (newAddress) {
    reconnect(newAddress)
    editingAddress.value = false
  }
}

const handleRetry = () => {
  reconnect(serverAddress.value)
}
</script>

<template>
  <div class="status-panel">
    <div class="connection-row">
      <div class="connection-status" :class="{ connected: connected, reconnecting: reconnecting }">
        <span class="indicator"></span>
        {{ reconnecting ? 'Reconnecting...' : (connected ? 'Connected' : 'Disconnected') }}
      </div>

      <button
        v-if="!connected && !reconnecting"
        class="retry-btn"
        @click="handleRetry"
        title="Retry connection"
      >
        Retry
      </button>
    </div>

    <div class="server-address">
      <template v-if="!editingAddress">
        <span class="address-label">Server:</span>
        <span class="address-value">{{ serverAddress }}</span>
        <button class="edit-btn" @click="startEditingAddress" title="Change server address">
          Edit
        </button>
      </template>
      <template v-else>
        <input
          v-model="addressInput"
          type="text"
          class="address-input"
          placeholder="Robot IP (e.g., 10.28.52.2)"
          @keyup.enter="submitAddress"
          @keyup.escape="cancelEditingAddress"
        />
        <button class="save-btn" @click="submitAddress">Connect</button>
        <button class="cancel-btn" @click="cancelEditingAddress">Cancel</button>
      </template>
    </div>

    <div class="calibration-enabled" :class="{ active: enabled }">
      <span class="indicator"></span>
      Calibration {{ enabled ? 'Active' : 'Inactive' }}
    </div>

    <div class="data-freshness" :class="{ stale: dataFreshness.stale, warning: dataFreshness.warning }">
      <span class="freshness-indicator"></span>
      Data: {{ dataFreshness.text }}
    </div>

    <div class="data-quality" :style="{ borderColor: dataQuality.color }">
      <span class="quality-indicator" :style="{ background: dataQuality.color }"></span>
      Coverage: {{ dataQuality.text }} ({{ store.completionPercentage }}%)
    </div>

    <div class="status-message">{{ status }}</div>

    <div v-if="error" class="error-message">{{ error }}</div>
  </div>
</template>

<style scoped>
.status-panel {
  background: #252540;
  border-radius: 8px;
  padding: 16px;
  display: flex;
  flex-direction: column;
  gap: 12px;
}

.connection-row {
  display: flex;
  align-items: center;
  gap: 12px;
}

.connection-status,
.calibration-enabled {
  display: flex;
  align-items: center;
  gap: 8px;
  font-size: 14px;
}

.indicator {
  width: 10px;
  height: 10px;
  border-radius: 50%;
  background: #666;
}

.connection-status.connected .indicator {
  background: #4caf50;
  box-shadow: 0 0 6px #4caf50;
}

.connection-status.reconnecting .indicator {
  background: #ff9800;
  box-shadow: 0 0 6px #ff9800;
  animation: pulse 1s infinite;
}

@keyframes pulse {
  0%, 100% { opacity: 1; }
  50% { opacity: 0.5; }
}

.calibration-enabled.active .indicator {
  background: #2196f3;
  box-shadow: 0 0 6px #2196f3;
}

.data-freshness {
  display: flex;
  align-items: center;
  gap: 8px;
  font-size: 13px;
  padding: 6px 8px;
  background: #1a1a2e;
  border-radius: 4px;
}

.freshness-indicator {
  width: 8px;
  height: 8px;
  border-radius: 50%;
  background: #4caf50;
}

.data-freshness.stale .freshness-indicator {
  background: #f44336;
}

.data-freshness.warning .freshness-indicator {
  background: #ff9800;
}

.data-quality {
  display: flex;
  align-items: center;
  gap: 8px;
  font-size: 13px;
  padding: 6px 8px;
  background: #1a1a2e;
  border-radius: 4px;
  border-left: 3px solid;
}

.quality-indicator {
  width: 8px;
  height: 8px;
  border-radius: 50%;
}

.retry-btn {
  padding: 4px 12px;
  border: 1px solid #ff9800;
  border-radius: 4px;
  background: transparent;
  color: #ff9800;
  font-size: 12px;
  cursor: pointer;
  transition: all 0.2s;
}

.retry-btn:hover {
  background: rgba(255, 152, 0, 0.1);
}

.server-address {
  display: flex;
  align-items: center;
  gap: 8px;
  font-size: 13px;
  padding: 8px;
  background: #1a1a2e;
  border-radius: 4px;
}

.address-label {
  color: #888;
}

.address-value {
  color: #ddd;
  font-family: 'Consolas', monospace;
}

.edit-btn,
.save-btn,
.cancel-btn {
  padding: 4px 8px;
  border: none;
  border-radius: 4px;
  font-size: 11px;
  cursor: pointer;
  transition: all 0.2s;
}

.edit-btn {
  background: #3f51b5;
  color: white;
  margin-left: auto;
}

.edit-btn:hover {
  background: #5c6bc0;
}

.save-btn {
  background: #4caf50;
  color: white;
}

.save-btn:hover {
  background: #66bb6a;
}

.cancel-btn {
  background: #666;
  color: white;
}

.cancel-btn:hover {
  background: #888;
}

.address-input {
  flex: 1;
  padding: 4px 8px;
  border: 1px solid #444;
  border-radius: 4px;
  background: #252540;
  color: #eee;
  font-size: 13px;
  font-family: 'Consolas', monospace;
}

.address-input:focus {
  outline: none;
  border-color: #3f51b5;
}

.status-message {
  color: #aaa;
  font-size: 13px;
  padding: 8px;
  background: #1a1a2e;
  border-radius: 4px;
}

.error-message {
  color: #f44336;
  font-size: 13px;
  padding: 8px;
  background: rgba(244, 67, 54, 0.1);
  border-radius: 4px;
  border: 1px solid rgba(244, 67, 54, 0.3);
}
</style>

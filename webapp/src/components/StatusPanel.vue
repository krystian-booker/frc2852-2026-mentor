<script setup lang="ts">
import { useNTBoolean, useNTString, useNTConnection } from '../composables/useNetworkTables'

const { connected } = useNTConnection()
const enabled = useNTBoolean('/TurretCalibration/Enabled', false)
const status = useNTString('/TurretCalibration/Status', 'Not Connected')
const error = useNTString('/TurretCalibration/Error', '')
</script>

<template>
  <div class="status-panel">
    <div class="connection-status" :class="{ connected: connected }">
      <span class="indicator"></span>
      {{ connected ? 'Connected' : 'Disconnected' }}
    </div>

    <div class="calibration-enabled" :class="{ active: enabled }">
      <span class="indicator"></span>
      Calibration {{ enabled ? 'Active' : 'Inactive' }}
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

.calibration-enabled.active .indicator {
  background: #2196f3;
  box-shadow: 0 0 6px #2196f3;
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

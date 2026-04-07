<script setup lang="ts">
defineProps<{
  title: string
  message: string
  confirmText?: string
  cancelText?: string
  danger?: boolean
}>()

const emit = defineEmits<{
  (e: 'confirm'): void
  (e: 'cancel'): void
}>()

const handleConfirm = () => {
  emit('confirm')
}

const handleCancel = () => {
  emit('cancel')
}

const handleBackdropClick = (event: MouseEvent) => {
  if (event.target === event.currentTarget) {
    emit('cancel')
  }
}
</script>

<template>
  <Teleport to="body">
    <div class="modal-backdrop" @click="handleBackdropClick">
      <div class="modal-content">
        <h3 class="modal-title">{{ title }}</h3>
        <p class="modal-message">{{ message }}</p>
        <div class="modal-actions">
          <button class="btn btn-secondary" @click="handleCancel">
            {{ cancelText || 'Cancel' }}
          </button>
          <button
            class="btn"
            :class="danger ? 'btn-danger' : 'btn-primary'"
            @click="handleConfirm"
          >
            {{ confirmText || 'Confirm' }}
          </button>
        </div>
      </div>
    </div>
  </Teleport>
</template>

<style scoped>
.modal-backdrop {
  position: fixed;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  background: rgba(0, 0, 0, 0.7);
  display: flex;
  align-items: center;
  justify-content: center;
  z-index: 1000;
}

.modal-content {
  background: #252540;
  border-radius: 12px;
  padding: 24px;
  min-width: 320px;
  max-width: 450px;
  border: 1px solid #3f51b5;
  box-shadow: 0 8px 32px rgba(0, 0, 0, 0.4);
}

.modal-title {
  margin: 0 0 12px 0;
  font-size: 18px;
  font-weight: 600;
  color: #fff;
}

.modal-message {
  margin: 0 0 24px 0;
  font-size: 14px;
  color: #aaa;
  line-height: 1.5;
}

.modal-actions {
  display: flex;
  gap: 12px;
  justify-content: flex-end;
}

.btn {
  padding: 10px 20px;
  border: none;
  border-radius: 6px;
  font-size: 14px;
  font-weight: 500;
  cursor: pointer;
  transition: all 0.2s;
}

.btn-secondary {
  background: #1a1a2e;
  color: #aaa;
  border: 1px solid #444;
}

.btn-secondary:hover {
  background: #2a2a4a;
  color: #fff;
}

.btn-primary {
  background: #3f51b5;
  color: white;
}

.btn-primary:hover {
  background: #5c6bc0;
}

.btn-danger {
  background: #f44336;
  color: white;
}

.btn-danger:hover {
  background: #e53935;
}
</style>

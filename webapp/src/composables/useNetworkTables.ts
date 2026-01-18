import { ref, onMounted, onUnmounted, type Ref } from 'vue'
import { NetworkTables, NetworkTablesTypeInfos, type NetworkTablesTopic } from 'ntcore-ts-client'

// Storage key for persisting server address
const STORAGE_KEY = 'nt-server-address'

// Get stored server address or default to localhost
const getStoredAddress = (): string => {
  try {
    return localStorage.getItem(STORAGE_KEY) || 'localhost'
  } catch {
    return 'localhost'
  }
}

// Store server address
const storeAddress = (address: string) => {
  try {
    localStorage.setItem(STORAGE_KEY, address)
  } catch {
    // Ignore storage errors
  }
}

// Shared state for server address and connection
const serverAddress = ref(getStoredAddress())
const connected = ref(false)
const reconnecting = ref(false)

// Current NT instance
let ntInstance = NetworkTables.getInstanceByURI(serverAddress.value)

// Cache for published topics to avoid re-publishing
let doubleTopics = new Map<string, NetworkTablesTopic<number>>()
let booleanTopics = new Map<string, NetworkTablesTopic<boolean>>()

// Subscription registry to track active subscriptions for reconnection
interface SubscriptionEntry {
  topicName: string
  type: 'boolean' | 'double' | 'integer' | 'string' | 'stringArray'
  callback: (value: any) => void
  ntTopic: NetworkTablesTopic<any> | null
  subscriberId: number | null
}

// Registry of all active subscriptions - keyed by unique ID
let subscriptionIdCounter = 0
const subscriptionRegistry = new Map<number, SubscriptionEntry>()

// Register a subscription and return an ID for unregistration
const registerSubscription = (
  topicName: string,
  type: SubscriptionEntry['type'],
  callback: (value: any) => void
): number => {
  const id = ++subscriptionIdCounter
  const entry: SubscriptionEntry = {
    topicName,
    type,
    callback,
    ntTopic: null,
    subscriberId: null
  }
  subscriptionRegistry.set(id, entry)

  // Subscribe immediately if we have an NT instance
  subscribeEntry(entry)

  return id
}

// Unregister a subscription
const unregisterSubscription = (id: number): void => {
  const entry = subscriptionRegistry.get(id)
  if (entry) {
    if (entry.ntTopic && entry.subscriberId !== null) {
      entry.ntTopic.unsubscribe(entry.subscriberId)
    }
    subscriptionRegistry.delete(id)
  }
}

// Subscribe a single entry to the current NT instance
const subscribeEntry = (entry: SubscriptionEntry): void => {
  const typeInfo = {
    'boolean': NetworkTablesTypeInfos.kBoolean,
    'double': NetworkTablesTypeInfos.kDouble,
    'integer': NetworkTablesTypeInfos.kInteger,
    'string': NetworkTablesTypeInfos.kString,
    'stringArray': NetworkTablesTypeInfos.kStringArray
  }[entry.type]

  entry.ntTopic = ntInstance.createTopic(entry.topicName, typeInfo)
  entry.subscriberId = entry.ntTopic.subscribe((newValue) => {
    if (newValue !== null) {
      entry.callback(newValue)
    }
  })
}

// Re-subscribe all registered subscriptions to a new NT instance
const resubscribeAll = (): void => {
  console.log(`NetworkTables: Re-subscribing ${subscriptionRegistry.size} topics`)
  for (const entry of subscriptionRegistry.values()) {
    // Cleanup old subscription
    if (entry.ntTopic && entry.subscriberId !== null) {
      try {
        entry.ntTopic.unsubscribe(entry.subscriberId)
      } catch {
        // Ignore errors from old instance
      }
    }
    // Create new subscription
    subscribeEntry(entry)
  }
}

// Connection listener cleanup
let connectionListenerCleanup: (() => void) | null = null

// Auto-reconnect state
let reconnectAttempts = 0
let reconnectTimer: ReturnType<typeof setTimeout> | null = null
const MAX_RECONNECT_DELAY_MS = 30000 // 30 seconds max
const BASE_RECONNECT_DELAY_MS = 1000 // 1 second base

// Calculate exponential backoff delay
const getReconnectDelay = (): number => {
  const delay = Math.min(BASE_RECONNECT_DELAY_MS * Math.pow(2, reconnectAttempts), MAX_RECONNECT_DELAY_MS)
  return delay
}

// Schedule a reconnection attempt
const scheduleReconnect = () => {
  if (reconnectTimer) {
    clearTimeout(reconnectTimer)
  }

  const delay = getReconnectDelay()
  console.log(`NetworkTables: Scheduling reconnect in ${delay}ms (attempt ${reconnectAttempts + 1})`)

  reconnectTimer = setTimeout(() => {
    reconnectAttempts++
    reconnecting.value = true

    // Clear topic caches and create new instance
    doubleTopics.clear()
    booleanTopics.clear()

    // Cleanup old connection listener
    if (connectionListenerCleanup) {
      connectionListenerCleanup()
      connectionListenerCleanup = null
    }

    // Create new NT instance with same address
    ntInstance = NetworkTables.getInstanceByURI(serverAddress.value)

    // Re-subscribe all active subscriptions to new instance
    resubscribeAll()

    // Setup new connection listener
    setupConnectionListener()
  }, delay)
}

// Setup connection listener
const setupConnectionListener = () => {
  if (connectionListenerCleanup) {
    connectionListenerCleanup()
  }
  connectionListenerCleanup = ntInstance.addRobotConnectionListener((isConnected) => {
    const wasConnected = connected.value
    connected.value = isConnected

    if (isConnected) {
      // Successfully connected - reset reconnect state
      reconnecting.value = false
      reconnectAttempts = 0
      if (reconnectTimer) {
        clearTimeout(reconnectTimer)
        reconnectTimer = null
      }
      console.log('NetworkTables: Connected')
    } else if (wasConnected) {
      // Lost connection - schedule reconnect
      console.log('NetworkTables: Connection lost, scheduling reconnect')
      scheduleReconnect()
    }
  }, true)
}

// Initialize connection listener
setupConnectionListener()

/**
 * Reconnects to a new server address.
 * Clears all topic caches and creates a new NT instance.
 */
export function reconnect(address: string): void {
  reconnecting.value = true

  // Store the new address
  serverAddress.value = address
  storeAddress(address)

  // Reset auto-reconnect state
  reconnectAttempts = 0
  if (reconnectTimer) {
    clearTimeout(reconnectTimer)
    reconnectTimer = null
  }

  // Clear topic caches
  doubleTopics.clear()
  booleanTopics.clear()

  // Cleanup old connection listener
  if (connectionListenerCleanup) {
    connectionListenerCleanup()
    connectionListenerCleanup = null
  }

  // Create new NT instance with new address
  ntInstance = NetworkTables.getInstanceByURI(address)

  // Re-subscribe all active subscriptions to new instance
  resubscribeAll()

  // Setup new connection listener
  setupConnectionListener()
}

export function useNTConnection() {
  return { connected, serverAddress, reconnecting, reconnect }
}

export function useNTBoolean(topic: string, defaultValue: boolean = false): Ref<boolean> {
  const value = ref(defaultValue)
  let registrationId: number | null = null

  onMounted(() => {
    registrationId = registerSubscription(topic, 'boolean', (newValue) => {
      value.value = newValue
    })
  })

  onUnmounted(() => {
    if (registrationId !== null) {
      unregisterSubscription(registrationId)
    }
  })

  return value
}

export function useNTDouble(topic: string, defaultValue: number = 0): Ref<number> {
  const value = ref(defaultValue)
  let registrationId: number | null = null

  onMounted(() => {
    registrationId = registerSubscription(topic, 'double', (newValue) => {
      value.value = newValue
    })
  })

  onUnmounted(() => {
    if (registrationId !== null) {
      unregisterSubscription(registrationId)
    }
  })

  return value
}

export function useNTInteger(topic: string, defaultValue: number = 0): Ref<number> {
  const value = ref(defaultValue)
  let registrationId: number | null = null

  onMounted(() => {
    registrationId = registerSubscription(topic, 'integer', (newValue) => {
      value.value = newValue
    })
  })

  onUnmounted(() => {
    if (registrationId !== null) {
      unregisterSubscription(registrationId)
    }
  })

  return value
}

export function useNTString(topic: string, defaultValue: string = ''): Ref<string> {
  const value = ref(defaultValue)
  let registrationId: number | null = null

  onMounted(() => {
    registrationId = registerSubscription(topic, 'string', (newValue) => {
      value.value = newValue
    })
  })

  onUnmounted(() => {
    if (registrationId !== null) {
      unregisterSubscription(registrationId)
    }
  })

  return value
}

export function useNTStringArray(topic: string, defaultValue: string[] = []): Ref<string[]> {
  const value = ref<string[]>(defaultValue)
  let registrationId: number | null = null

  onMounted(() => {
    registrationId = registerSubscription(topic, 'stringArray', (newValue) => {
      value.value = newValue
    })
  })

  onUnmounted(() => {
    if (registrationId !== null) {
      unregisterSubscription(registrationId)
    }
  })

  return value
}

async function getOrCreateDoubleTopic(topic: string): Promise<NetworkTablesTopic<number>> {
  if (!doubleTopics.has(topic)) {
    const ntTopic = ntInstance.createTopic<number>(topic, NetworkTablesTypeInfos.kDouble)
    await ntTopic.publish()
    doubleTopics.set(topic, ntTopic)
  }
  return doubleTopics.get(topic)!
}

async function getOrCreateBooleanTopic(topic: string): Promise<NetworkTablesTopic<boolean>> {
  if (!booleanTopics.has(topic)) {
    const ntTopic = ntInstance.createTopic<boolean>(topic, NetworkTablesTypeInfos.kBoolean)
    await ntTopic.publish()
    booleanTopics.set(topic, ntTopic)
  }
  return booleanTopics.get(topic)!
}

export async function publishDouble(topic: string, value: number): Promise<void> {
  const ntTopic = await getOrCreateDoubleTopic(topic)
  ntTopic.setValue(value)
}

export async function publishBoolean(topic: string, value: boolean): Promise<void> {
  const ntTopic = await getOrCreateBooleanTopic(topic)
  ntTopic.setValue(value)
}

export async function triggerBoolean(topic: string): Promise<void> {
  const ntTopic = await getOrCreateBooleanTopic(topic)
  ntTopic.setValue(true)
  // Reset after a short delay for edge-triggered topics
  // 150ms provides safety margin for robot running at 50Hz (20ms/loop)
  // Return promise that resolves after reset completes
  return new Promise((resolve) => {
    setTimeout(() => {
      ntTopic.setValue(false)
      resolve()
    }, 150)
  })
}

export { ntInstance }

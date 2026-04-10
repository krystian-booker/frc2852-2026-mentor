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

interface DoubleTopicState {
  ntTopic: NetworkTablesTopic<number>
  publishPromise: Promise<NetworkTablesTopic<number>> | null
  writePromise: Promise<void>
}

// Per-topic writer state so writes are serialized and publisher acquisition
// cannot race against concurrent retries for the same topic.
let doubleTopics = new Map<string, DoubleTopicState>()

// Subscription registry to track active subscriptions for reconnection
interface SubscriptionEntry {
  topicName: string
  type: 'boolean' | 'double' | 'integer' | 'string'
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
    'string': NetworkTablesTypeInfos.kString
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

// Setup connection listener
const setupConnectionListener = () => {
  if (connectionListenerCleanup) {
    connectionListenerCleanup()
  }
  connectionListenerCleanup = ntInstance.addRobotConnectionListener((isConnected) => {
    const wasConnected = connected.value
    if (isConnected === wasConnected) {
      return
    }
    connected.value = isConnected

    if (isConnected) {
      reconnecting.value = false
      console.log('NetworkTables: Connected')
    } else if (wasConnected) {
      reconnecting.value = true
      console.log('NetworkTables: Connection lost')
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

  // Clear topic caches
  doubleTopics.clear()

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

function getOrCreateDoubleTopicState(topic: string): DoubleTopicState {
  const existingState = doubleTopics.get(topic)
  if (existingState) {
    return existingState
  }

  const ntTopic = ntInstance.createTopic<number>(topic, NetworkTablesTypeInfos.kDouble)
  const state: DoubleTopicState = {
    ntTopic,
    publishPromise: null,
    writePromise: Promise.resolve()
  }
  doubleTopics.set(topic, state)
  return state
}

async function waitForPublisher(topic: NetworkTablesTopic<number>, timeoutMs: number = 500): Promise<boolean> {
  if (topic.publisher) {
    return true
  }

  const start = Date.now()
  while (Date.now() - start < timeoutMs) {
    await new Promise<void>((resolve) => globalThis.setTimeout(resolve, 25))
    if (topic.publisher) {
      return true
    }
  }

  return topic.publisher
}

async function ensureDoubleTopicPublished(state: DoubleTopicState): Promise<NetworkTablesTopic<number>> {
  if (state.ntTopic.publisher) {
    return state.ntTopic
  }

  if (!state.publishPromise) {
    state.publishPromise = (async () => {
      try {
        try {
          await state.ntTopic.publish()
        } catch (error) {
          // ntcore-ts-client can time out waiting for an announce even when the
          // topic eventually becomes the publisher. Give the announce path a
          // short window to land before treating this as a real failure.
          console.warn(`NetworkTables: publish wait failed for ${state.ntTopic.name}`, error)
        }

        if (!(await waitForPublisher(state.ntTopic))) {
          throw new Error(`Topic ${state.ntTopic.name} did not acquire publisher ownership`)
        }

        return state.ntTopic
      } finally {
        state.publishPromise = null
      }
    })()
  }

  return state.publishPromise
}

export async function publishDouble(topic: string, value: number): Promise<void> {
  const state = getOrCreateDoubleTopicState(topic)

  state.writePromise = state.writePromise
    .catch(() => {
      // Preserve the write queue after an earlier failed attempt.
    })
    .then(async () => {
      let ntTopic = await ensureDoubleTopicPublished(state)

      try {
        ntTopic.setValue(value)
      } catch (error) {
        console.warn(`NetworkTables: setValue failed for ${topic}, retrying publish once`, error)
        ntTopic = await ensureDoubleTopicPublished(state)
        ntTopic.setValue(value)
      }
    })

  return state.writePromise
}

def captureState(bool, ref):
    capture_state_ref = ref.child("captureState")
    capture_state_ref.set(bool)
    return None

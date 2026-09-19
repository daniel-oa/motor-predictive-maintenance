#  RuntimeBaseline: runtime normaliser for the Pi 
class RuntimeBaseline:
    def __init__(self, seed=None):
        if seed is not None:
            # pre-seeded: ready immediately
            self._base   = {'i': max(seed['Irms'], 1e-6),
                            'v': max(seed['Vib'],  1e-6),
                            't': max(seed['Temp'], 1e-6)}
            self._ready  = True
        else:
            self._base   = None
            self._ready  = False
 
    def is_ready(self):
        return self._ready

    def force_set(self, irms, vib_rms, temp_rise):
        self._base  = {
            'i': max(irms,     1e-6),
            'v': max(vib_rms,  1e-6),
            't': max(temp_rise,1e-6),
        }
        self._ready  = True
        print(f"Baseline force-set: Irms={irms:.4f} Vib={vib_rms:.4f} Temp={temp_rise:.4f}")

    def reset(self):
        """Call this when a new motor is connected."""
        self.__init__()
        
    def compute_ratios(self, irms, vib_rms, temp_rise):
        """Compute ratios against current baseline without updating it."""
        if not self._ready:
            return None
        ir = max(irms,     1e-6) / self._base['i']
        vr = max(vib_rms,  1e-6) / self._base['v']
        tr = max(temp_rise,1e-6) / self._base['t']
        return {
            'Irms_ratio': ir,
            'Vib_ratio':  vr,
            'Temp_ratio': tr,
        }
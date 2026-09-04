"""Re-export shim, not a second copy -- deployment_package.joblib's pickle
also references `battlib.online.SteadyStateGate` (the package's `cc_gate`
entry), so that exact import path must resolve too, same reason
battlib/models.py exists. The real, maintained vendored copy is
soh/online.py; this module just points at it so both pickle paths and normal
APP3_0 code (`from soh.online import ...`) refer to the same class objects.
"""

from soh.online import *  # noqa: F401,F403
from soh.online import SteadyStateGate  # noqa: F401

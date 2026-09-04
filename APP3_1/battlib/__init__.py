"""Package name required for joblib.load(deployment_package.joblib) to
unpickle -- the file was saved from C:\\code2\\python's battlib.models.Ffnn,
so that exact import path must resolve for unpickling to work at all. This is
NOT a copy of the full battlib project; see battlib/models.py's own docstring
for exactly what's vendored and why. The rest of APP3_0's SoH code lives in
soh/, not here -- this package exists purely to satisfy the pickle.
"""

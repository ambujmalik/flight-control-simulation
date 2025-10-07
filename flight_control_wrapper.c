// flight_control_wrapper.c
#include <Python.h>
#include <numpy/arrayobject.h>
#include "flight_control_asm.h"

static PyObject* py_compute_dynamic_pressure(PyObject* self, PyObject* args) {
    double rho, V;
    if (!PyArg_ParseTuple(args, "dd", &rho, &V)) {
        return NULL;
    }
    
    double result = compute_dynamic_pressure_asm(rho, V);
    return PyFloat_FromDouble(result);
}

static PyObject* py_compute_forces_moments(PyObject* self, PyObject* args) {
    PyObject *params_obj, *state_obj, *controls_obj;
    PyArrayObject *params_array, *state_array, *controls_array;
    
    if (!PyArg_ParseTuple(args, "OOO", &params_obj, &state_obj, &controls_obj)) {
        return NULL;
    }
    
    // Convert to numpy arrays
    params_array = (PyArrayObject*)PyArray_FROM_OTF(params_obj, NPY_DOUBLE, NPY_ARRAY_IN_ARRAY);
    state_array = (PyArrayObject*)PyArray_FROM_OTF(state_obj, NPY_DOUBLE, NPY_ARRAY_IN_ARRAY);
    controls_array = (PyArrayObject*)PyArray_FROM_OTF(controls_obj, NPY_DOUBLE, NPY_ARRAY_IN_ARRAY);
    
    if (params_array == NULL || state_array == NULL || controls_array == NULL) {
        Py_XDECREF(params_array);
        Py_XDECREF(state_array);
        Py_XDECREF(controls_array);
        return NULL;
    }
    
    // Create result array
    npy_intp dims[1] = {6};
    PyArrayObject *result_array = (PyArrayObject*)PyArray_SimpleNew(1, dims, NPY_DOUBLE);
    
    if (result_array == NULL) {
        Py_DECREF(params_array);
        Py_DECREF(state_array);
        Py_DECREF(controls_array);
        return NULL;
    }
    
    // Call assembly function
    compute_forces_moments_asm(
        (double*)PyArray_DATA(params_array),
        (double*)PyArray_DATA(state_array),
        (double*)PyArray_DATA(controls_array),
        (double*)PyArray_DATA(result_array)
    );
    
    Py_DECREF(params_array);
    Py_DECREF(state_array);
    Py_DECREF(controls_array);
    
    return (PyObject*)result_array;
}

static PyMethodDef FlightControlMethods[] = {
    {"compute_dynamic_pressure", py_compute_dynamic_pressure, METH_VARARGS,
     "Compute dynamic pressure using assembly optimization"},
    {"compute_forces_moments", py_compute_forces_moments, METH_VARARGS,
     "Compute forces and moments using assembly optimization"},
    {NULL, NULL, 0, NULL}
};

static struct PyModuleDef flightcontrolmodule = {
    PyModuleDef_HEAD_INIT,
    "flight_control_asm",
    "Assembly-optimized flight control functions",
    -1,
    FlightControlMethods
};

PyMODINIT_FUNC PyInit_flight_control_asm(void) {
    import_array();
    return PyModule_Create(&flightcontrolmodule);
}

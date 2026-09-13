// flight_control_wrapper.c
#include <Python.h>
#include <numpy/arrayobject.h>
#include "flight_control_asm.h"

// Validate array dimensions and type
static int validate_array(PyArrayObject* arr, int expected_size, const char* name) {
    if (arr == NULL) {
        PyErr_Format(PyExc_TypeError, "%s: array is NULL", name);
        return 0;
    }
    
    if (PyArray_TYPE(arr) != NPY_DOUBLE) {
        PyErr_Format(PyExc_TypeError, 
                    "%s: expected float64 array, got %s",
                    name, PyArray_DescrFromScalar(PyArray_DESCR(arr))->typestr);
        return 0;
    }
    
    if (PyArray_SIZE(arr) < expected_size) {
        PyErr_Format(PyExc_ValueError,
                    "%s: expected array size >= %d, got %ld",
                    name, expected_size, PyArray_SIZE(arr));
        return 0;
    }
    
    return 1;
}

static PyObject* py_compute_dynamic_pressure(PyObject* self, PyObject* args) {
    double rho, V;
    if (!PyArg_ParseTuple(args, "dd", &rho, &V)) {
        return NULL;
    }
    
    if (rho <= 0.0 || V < 0.0) {
        PyErr_SetString(PyExc_ValueError, 
                       "rho must be positive and V must be non-negative");
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
    
    // Validate array sizes
    // params: [rho, CL0, CLa, CLq, MAC, CLde, S, CD0, K, CDde, CY_beta, CY_dr, 
    //          Cm0, Cm_alpha, Cm_q, Cm_de, Cl_beta, Cl_p, Cl_r, Cl_da, Cl_dr,
    //          Cn_beta, Cn_p, Cn_r, Cn_da, Cn_dr] = 26 elements
    // state: [V, alpha, beta, p, q, r, phi, theta, psi, pos_n, pos_e, pos_d] = 12 elements
    // controls: [delta_e, delta_a, delta_r, delta_t] = 4 elements
    
    if (!validate_array(params_array, 26, "params")) {
        Py_DECREF(params_array);
        Py_DECREF(state_array);
        Py_DECREF(controls_array);
        return NULL;
    }
    
    if (!validate_array(state_array, 12, "state")) {
        Py_DECREF(params_array);
        Py_DECREF(state_array);
        Py_DECREF(controls_array);
        return NULL;
    }
    
    if (!validate_array(controls_array, 4, "controls")) {
        Py_DECREF(params_array);
        Py_DECREF(state_array);
        Py_DECREF(controls_array);
        return NULL;
    }
    
    // Create result array [Fx, Fy, Fz, Mx, My, Mz]
    npy_intp dims[1] = {6};
    PyArrayObject *result_array = (PyArrayObject*)PyArray_SimpleNew(1, dims, NPY_DOUBLE);
    
    if (result_array == NULL) {
        Py_DECREF(params_array);
        Py_DECREF(state_array);
        Py_DECREF(controls_array);
        PyErr_SetString(PyExc_MemoryError, "Failed to allocate result array");
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

static PyObject* py_matrix_multiply_4x4(PyObject* self, PyObject* args) {
    PyObject *A_obj, *B_obj;
    PyArrayObject *A_array, *B_array;
    
    if (!PyArg_ParseTuple(args, "OO", &A_obj, &B_obj)) {
        return NULL;
    }
    
    // Convert to numpy arrays
    A_array = (PyArrayObject*)PyArray_FROM_OTF(A_obj, NPY_DOUBLE, NPY_ARRAY_IN_ARRAY);
    B_array = (PyArrayObject*)PyArray_FROM_OTF(B_obj, NPY_DOUBLE, NPY_ARRAY_IN_ARRAY);
    
    if (A_array == NULL || B_array == NULL) {
        Py_XDECREF(A_array);
        Py_XDECREF(B_array);
        return NULL;
    }
    
    // Validate array dimensions (must be 4x4 = 16 elements for 1D, or 2D 4x4)
    if (!validate_array(A_array, 16, "A") || !validate_array(B_array, 16, "B")) {
        Py_DECREF(A_array);
        Py_DECREF(B_array);
        return NULL;
    }
    
    // Create result array (4x4 = 16 elements)
    npy_intp dims[1] = {16};
    PyArrayObject *C_array = (PyArrayObject*)PyArray_SimpleNew(1, dims, NPY_DOUBLE);
    
    if (C_array == NULL) {
        Py_DECREF(A_array);
        Py_DECREF(B_array);
        PyErr_SetString(PyExc_MemoryError, "Failed to allocate result array");
        return NULL;
    }
    
    // Call assembly function
    matrix_multiply_4x4_asm(
        (double*)PyArray_DATA(A_array),
        (double*)PyArray_DATA(B_array),
        (double*)PyArray_DATA(C_array)
    );
    
    Py_DECREF(A_array);
    Py_DECREF(B_array);
    
    return (PyObject*)C_array;
}

static PyObject* py_quaternion_rotate(PyObject* self, PyObject* args) {
    PyObject *quat_obj, *vec_obj;
    PyArrayObject *quat_array, *vec_array;
    
    if (!PyArg_ParseTuple(args, "OO", &quat_obj, &vec_obj)) {
        return NULL;
    }
    
    // Convert to numpy arrays
    quat_array = (PyArrayObject*)PyArray_FROM_OTF(quat_obj, NPY_DOUBLE, NPY_ARRAY_IN_ARRAY);
    vec_array = (PyArrayObject*)PyArray_FROM_OTF(vec_obj, NPY_DOUBLE, NPY_ARRAY_IN_ARRAY);
    
    if (quat_array == NULL || vec_array == NULL) {
        Py_XDECREF(quat_array);
        Py_XDECREF(vec_array);
        return NULL;
    }
    
    // Validate array sizes
    if (!validate_array(quat_array, 4, "quat") || !validate_array(vec_array, 3, "vec")) {
        Py_DECREF(quat_array);
        Py_DECREF(vec_array);
        return NULL;
    }
    
    // Create result array (3-element vector)
    npy_intp dims[1] = {3};
    PyArrayObject *result_array = (PyArrayObject*)PyArray_SimpleNew(1, dims, NPY_DOUBLE);
    
    if (result_array == NULL) {
        Py_DECREF(quat_array);
        Py_DECREF(vec_array);
        PyErr_SetString(PyExc_MemoryError, "Failed to allocate result array");
        return NULL;
    }
    
    // Call assembly function
    quaternion_rotate_asm(
        (double*)PyArray_DATA(quat_array),
        (double*)PyArray_DATA(vec_array),
        (double*)PyArray_DATA(result_array)
    );
    
    Py_DECREF(quat_array);
    Py_DECREF(vec_array);
    
    return (PyObject*)result_array;
}

static PyMethodDef FlightControlMethods[] = {
    {"compute_dynamic_pressure", py_compute_dynamic_pressure, METH_VARARGS,
     "Compute dynamic pressure using assembly optimization\n"
     "compute_dynamic_pressure(rho: float, V: float) -> float\n"
     "Parameters:\n"
     "  rho: air density (kg/m^3), must be > 0\n"
     "  V: velocity (m/s), must be >= 0\n"
     "Returns:\n"
     "  Q: dynamic pressure = 0.5 * rho * V^2 (Pa)"},
    
    {"compute_forces_moments", py_compute_forces_moments, METH_VARARGS,
     "Compute aerodynamic forces and moments using assembly optimization\n"
     "compute_forces_moments(params: ndarray, state: ndarray, controls: ndarray) -> ndarray\n"
     "Parameters:\n"
     "  params: 26-element array of aerodynamic coefficients\n"
     "  state: 12-element array [V, alpha, beta, p, q, r, phi, theta, psi, pos_n, pos_e, pos_d]\n"
     "  controls: 4-element array [delta_e, delta_a, delta_r, delta_t]\n"
     "Returns:\n"
     "  result: 6-element array [Fx, Fy, Fz, Mx, My, Mz]"},
    
    {"matrix_multiply_4x4", py_matrix_multiply_4x4, METH_VARARGS,
     "Multiply two 4x4 matrices using assembly optimization\n"
     "matrix_multiply_4x4(A: ndarray, B: ndarray) -> ndarray\n"
     "Parameters:\n"
     "  A: 16-element array (4x4 matrix)\n"
     "  B: 16-element array (4x4 matrix)\n"
     "Returns:\n"
     "  C: 16-element array (4x4 result matrix)"},
    
    {"quaternion_rotate", py_quaternion_rotate, METH_VARARGS,
     "Rotate a vector using quaternion using assembly optimization\n"
     "quaternion_rotate(quat: ndarray, vec: ndarray) -> ndarray\n"
     "Parameters:\n"
     "  quat: 4-element array [w, x, y, z] (must be normalized)\n"
     "  vec: 3-element array [x, y, z]\n"
     "Returns:\n"
     "  result: 3-element rotated vector"},
    
    {NULL, NULL, 0, NULL}
};

static struct PyModuleDef flightcontrolmodule = {
    PyModuleDef_HEAD_INIT,
    "flight_control_asm",
    "Assembly-optimized flight control functions for x86-64\n"
    "Provides high-performance kernels for aerodynamic calculations",
    -1,
    FlightControlMethods
};

PyMODINIT_FUNC PyInit_flight_control_asm(void) {
    import_array();
    return PyModule_Create(&flightcontrolmodule);
}

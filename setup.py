# setup.py
from setuptools import setup, Extension
import numpy as np
import os

# Assembly source files
asm_sources = ['flight_control_asm.asm']
c_sources = ['flight_control_wrapper.c']

# Compile assembly files first
for asm_file in asm_sources:
    base_name = os.path.splitext(asm_file)[0]
    obj_file = base_name + '.o'
    
    # Assemble with NASM
    os.system(f'nasm -f elf64 -o {obj_file} {asm_file}')
    c_sources.append(obj_file)

extension = Extension(
    'flight_control_asm',
    sources=c_sources,
    include_dirs=[np.get_include()],
    extra_compile_args=['-O3', '-march=native'],
    extra_link_args=[],
)

setup(
    name='flight_control_asm',
    version='1.0',
    description='Assembly-optimized flight control functions',
    ext_modules=[extension],
)

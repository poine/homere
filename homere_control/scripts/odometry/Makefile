
cpp_homere_control.so: homere_control.pyx setup_cpp_ext.py
	python setup_cpp_ext.py build_ext --inplace

clean:
	rm -f *~ *.pyc homere_control.cpp cpp_homere_control.so

README file for libcpp folder
=============================

The folder contains files from GNU libsupc++. THEY MUST BE KEP UP-TO-DATE WITH THE ONES IN THE libsupc++ LIBRARY!!!

File eh-alloc.cpp is a modified version of the original GNU one which doesn't maintain an emergency buffer tool for
allocating exception objects if allocation using malloc() mfails. This saves more than 2K dynamic RAM.

File unwind-cxx.h is an include file needed by eh_alloc.cpp, unmodified from the original.

DC 2020-01-10

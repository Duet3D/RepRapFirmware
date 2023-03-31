README file for libcpp folder
=============================

The folder contains files from GNU libsupc++. THEY MUST BE KEP UP-TO-DATE WITH THE ONES IN THE libsupc++ LIBRARY!!!

File eh-alloc.cc is a modified version of the original GNU one which doesn't maintain an emergency buffer tool for
allocating exception objects if allocation using malloc() mfails. This saves more than 2K dynamic RAM.

File unwind-cxx.h is an include file needed by eh_alloc.cpp, unmodified from the original.

File vterminate.cc is a replacement for the original. The original attempts to print information about the terminating exception,
which pulls in fputs(), which we don't have/don't want.

DC 2020-01-10, updated 2023-03-31

# TRAIN PROPULSION SIMULATOR

## Overview
Train Propulsion Simulator (TPS) is a program for modeling longitudinal train dynamics.  TPS is written in the C++ programming language.  Currently, testing has only been performed on Microsoft Windows-based computers.  However, it is likely that only minor changes would be needed to run TPS on a Linux machine.

## User Manual
The most recent version of the user manual is contained in a word processing OpenDocument file located in the folder titled 'user_manual'.  A PDF version of the manual is also contained in the same folder.  The user manual contains multiple screen captures from an example TPS input file.  This example TPS input file titled 'My_TPS_Input_File.txt' can be found in the folder titled 'input_file_examples'.

## Generating TPS Executable File
When building the source code to create a TPS executable file, there are two extra steps that must be taken, in addition to the standard steps for building source code.  These two extra steps are detailed in the numbered bullet points below.
1. There are multiple .cpp files with a 'main' function.  Specifically, all 19 files in the 'TestFiles' folder (in other words, all source files ending with '__TEST') contain a 'main' function. However, these are files that were used for testing various components of TPS during development and do not contain the desired 'main' function.  Therefore, if using Microsoft Visual Studio, select these 19 files in the Solution Explorer window and then right-click and select 'Properties'.  In the 'Properties' window, under 'Configuration Properties' and then 'General', there is an option that says 'Excluded From Build'.  Set this option to 'Yes' for all 19 files in the 'TestFiles' folder.  The Session_Function.cpp file and the Session_Simulation.cpp file also both contain a 'main' function.  To create a TPS executable, the 'main' function in the Session_Simulation.cpp file is the desired 'main' function.  Therefore, right-click on the Session_Function.cpp file and follow the same steps outlined previously for the 19 files in the 'TestFiles' folder in order to exclude the Session_Function.cpp file from the build.
2. The code makes use of 'fopen' rather than 'fopen_s' for writing to files.  This is to provide greater compatibility with non-Windows operating systems, such as Linux.  If building the source code with Microsoft Visual Studio, a warning will likely appear stating that the 'fopen' function may be unsafe.  The following link on Stack Overflow explains how to bypass this warning: https://stackoverflow.com/questions/21873048/getting-an-error-fopen-this-function-or-variable-may-be-unsafe-when-complin/21873153.

## Generating TPS Function Visualizer Executable File
TPS includes a secondary/auxiliary program that allows users to visualize the functions being input into TPS.  The details of this function visualizer are put forth in Appendix A of the user manual.  The same two additional steps as outlined in the 'Generating TPS Executable File' outlined above must be taken.  However, in this case, the desired 'main' function is in the Session_Function.cpp file.  Therefore, right-click on the Session_Simulation.cpp file and select 'Properties'.  In the 'Properties' window, under 'General' there is an option that says 'Excluded From Build'.  Set this option to 'Yes'.  For the Session_Function.cpp file make sure that this same option is set to 'No'.

## Questions, Comments, and Bug Reporting
Questions and comments related to the TPS manual or program can be submitted as an issue.
Bugs can also be submitted as an issue.  Please be as specific as possible.  Also, providing the text of the TPS input file that generated the bug is helpful.

## Potential Improvements
There are many improvements that could be made to TPS including, but not limited to, the following:
1. Development of a graphical user interface to assist users in generating the TPS input file.
2. Development of a graphical user interface for viewing and analyzing the numerical results generated from running a TPS simulation.
3. Addressing the quirks and limitations of TPS put forth in the "Important TPS Quirks and Limitations" section of the user manual.

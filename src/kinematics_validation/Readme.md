# Running instructions and description:

1. cd to this directory in a terminal
2. This package contains 2 files, one which generates a symbolic FK matrix  from which transforms EEF(End Effector) pose and orientation to robot's base
3. The other file aims to draw a circle using the End effector, for which it generates a circular trajectory of the EEF, and also generates a joint trajectory which will draw this circle. It employs geometric jacobian based velocity IK and numerical integration, to get joint angles.
4. To generate Symbolic FK equation, type "python3 symbolicFk.py" 
5. To generate a circular trajectory using geometric jacobian , run "python3 ikValidation.py"
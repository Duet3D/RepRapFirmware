How to add support for additional kinematics to RepRapFirmware
==============================================================

The supported kinematics are defined in files in folder src/Movement/Kinematics. It is organised as a class hierarchy. Each supported kinematics has its own class definition. For example, class CartesianKinematics supports standard Cartesian motion kinematics. The class declaration is in file CartesianKinematics.h and the implementation is in class CartesianKinematics.cpp.

To add new kinematics:
1. Tell me the name of your kinematics and ask me (dc42) to allocate a kinematics type number for it, via the Duet3d forum. The kinematics type number will be the K parameter in the M669 command.

2. Create .h and .cpp files to declare and implement the class for your kinematics. If you want to support bed levelling using multiple independent leadscrews, or the facility to assist users in determining corrections to make to manual bed levelling screws, then you should derive your kinematics class from class ZLeadscrewKinematics. Otherwise it is normally appropriate to derive it directly from class Kinematics.

3. In your kinematics class, override virtual functions as needed. See the comments in file Kinematics.h for a description of those functions.

4. Modify file Kinematics.h by adding a #include directive to include the .h file that declares your new kinematics. Also modify function Create by adding a new case to create an instance of your new kinematics class when the appropriate kinematics type number is passed.

DC updated 2018-03-20.

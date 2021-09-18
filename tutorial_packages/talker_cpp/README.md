## The talker_cpp package

This folder is what's considered a "catkin package". A package is loosely speaking any folder that contains the package.xml and CMakeLists.txt files,
with valid "catkin package" contents. The files here can be used as a basis for new packages whenever you need to make one, especially if you plan to
write your package in C++. (In reality there are no restrictions on language for a package, but we tend to stick to writing one specific package in
one specific language)

To simplify the package creation process, you may run 

```
catkin_create_pkg <package_name>
```

which will create a folder with name <package_name> with a template package.xml and CMakeLists.txt. Note that these contain a lot of unneccesary components
(most of them are commented out, but it's a good idea to remove everything that's not used, so the files here may still come in handy). The CMakeLists.txt also contains a summary of the documentation that comes with the templates, and should be a bit easier to follow.

Since this is a package with C++ as the primary language, it contains the src/ and include/ folders, which need to be linked to in CMakeLists.txt!
Following standard C++ practices, header files go in the include/ folder, while source code goes in src/. Note that the headers are not placed directly in include/, but rather in include/talker_cpp/. This is standard practice for all catkin packages using C++.
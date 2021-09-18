## The listener package

This folder is what's considered a "catkin package". A package is loosely speaking any folder that contains the package.xml and CMakeLists.txt files,
with valid "catkin package" contents. The files here can be used as a basis for new packages whenever you need to make one, especially if you plan to
write your package in Python. (In reality there are no restrictions on language for a package, but we tend to stick to writing one specific package in
one specific language)

To simplify the package creation process, you may run 

```
catkin_create_pkg <package_name>
```

which will create a folder with name <package_name> with a template package.xml and CMakeLists.txt. Note that these contain a lot of unneccesary components
(most of them are commented out, but it's a good idea to remove everything that's not used, so the files here may still come in handy). For this package, the CMakeLists.txt and package.xml also contain a summary of the documentation that comes with the templates, and should be a bit easier to follow.

Since this is a package with Python as the primary language, all source files are put in the /scripts folder. Note that to be able to execute any of these scripts, you will need to run the command

```
chmod +x <filename>
```

This package also contains the /launch folder. Any launchfiles used for testing the node should be put here. If the package requires some complicated launch scheme, a separate launchfile should also be written and put here. This will simplify the launch process if others wish to use your package.
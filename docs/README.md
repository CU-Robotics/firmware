# Documentation Guidelines
We utilize [Doxygen](https://www.doxygen.nl/index.html) to autogenerate documentation from your code. 

Documentation from accepted code on `main` is hosted on github pages at [cu-robotics.github.io/firmware](cu-robotics.github.io/firmware). You can also compile the documentation locally by reading below.

This README describe how to install Doxygen, write documentation following our standards, and accessing your documentation once complete.

## Installing and Building Doxygen
You can find Doxygen's getting started guide here: [https://www.doxygen.nl/manual/index.html](https://www.doxygen.nl/manual/index.html). It provides download links for the documentation compiler and a guide on how to use doxygen [here](https://www.doxygen.nl/manual/install.html). 
- There are additional installs for latex support: https://www.tug.org/texlive/
- Class graphs and diagrams from graphviz: https://www.graphviz.org/
Make sure to install both of these dependencies as well.

We have configured a Doxyfile containing our configuration. To compile (with doxygen installed), simply run `doxygen Doxyfile-build` or `doxygen Doxyfile-test` at the root of your branch.
- `Doxyfile-build` builds doxygen docs without failing on undocumented warnings, and includes private members.
- `Doxyfile-test` is what you use to test if your documentation is complete. This will give you warnings for any documentation you may have missed in [doxygen_warnings.txt](./doxygen_warnings.txt).

Then, to view the generated docs, open the file [file:///{project_root}/docs/html/index.html](./html/index.html) in your browser as a local html website. 

## Standards

When you write documentation, ask yourself two questions:
- What is \<thing>'s purpose?
- What do I expect to happen when I use \<thing>?

Keep it high level, but be clear. Include any information that you think might help you or someone else later.

You must document all your public functions, objects, variables, structs, etc.
- Private members are not required to be documented, but any documentation on these could help understanding the code later.

See the [Doxygen docblocks manual](https://www.doxygen.nl/manual/docblocks.html) for information on different tags. Some important ones include:
- `@brief`: Provide a summary of purpose and behavior.
- `@param <arg_name>`: What is expected from this argument? (do this for every argument)
- `@return`: What is returned (if not void)?
- `@note` Any additional notable information to add.
- Doxygen has many more features, feel free to use them if you think it will help to better represent your code.

We're using three `///` slashes, with `@` to represent the specific strings:
Here's a few examples: (Most documentation belongs in the hpp header file)

```cpp
    /// @brief Get the temperature of the sensor
    /// @return temperature in Celcius
    float get_temperature();

    /// @brief Get the acceleration of the sensor in its local x axis.
    /// @return acceleration m/s^2
    float get_accel_X();

    /// @brief Sets the output array message for the corresponding motor
    /// @note Does not issue a Write command to the CANs
    /// @param canID ID of the CAN which the motor is on, expects an indexable ID value
    /// @param value A value in the range of [-16384,16384] representing ampage to send to the motor.
    void write_motor(uint16_t canID, uint16_t motorID, int32_t value)

```

## That's it!

Be sure to ask any questions for additional clarification.

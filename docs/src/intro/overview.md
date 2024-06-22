# Package Overview

*This page is a work-in-progress and will be updated later*

## Models with Named Access

Many MuJoCo models attach names to the various joints and bodies that make up the model. It is often convenient to select a single part of the body and access information that may be contained in the larger arrays of `model` or `data`. `MuJoCo.jl` provides a few wrapper functions that allow us to access the underlying data.

As an example, let's investigate the torso of our humanoid. If you display the `torso` object in the REPL, you can see the fields that can be accessed in this object.
```@example torso
import MuJoCo as MJ
model, data = MJ.sample_model_and_data();
MJ.step!(model, data);
torso = MJ.body(data, "torso")
```
For example, `torso.com` will give you a 3-dimensional vector of the torso's centre of mass coordinates. 
```@example torso
torso.com
```
All other functions that generate a named wrapper around an object are documented in [Named Access](@ref).

## Row vs. Column-Major Arrays

MuJoCo's underlying C API expects arrays to be **row-major**, while by default, all Julia arrays are **column-major**. However, Julia is high level enough to allow us to mimic the use of a row-major array by changing the type of the object, while keeping the array access patterns the same.

We have included [`mj_array`](@ref) and [`mj_zeros`](@ref) to make life easier when passing arrays to MuJoCo functions that directly mutate their arguments (eg: [`mjd_transitionFD`](@ref) and many others). An array initialised with either [`mj_array`](@ref) or [`mj_zeros`](@ref) will be a `transpose(<:AbstractArray)` and can be treated just the same as any other Julia array, but its memory will be accessed in **row-major** order. This means that the C API will correctly fill your arrays. Using these functions also still allows for performance specialisation based on the type. 

Let's look at an example. Suppose we want to compute a Jacobian for the humanoid's torso. There are `model.nv` degrees of freedom in the humanoid, and the torso's centre of mass is a 3-element vector. The Jacobian should therefore be a `3 x nv` array. To ensure the order of elements is correct, we compute it with
```@example torso
jac_torso = MJ.mj_zeros(3, model.nv)
MJ.mj_jacSubtreeCom(model, data, jac_torso, torso.id)
@show jac_torso
```

!!! warning "Beware the row-major order"
    Passing a `3 x nv` array directly to `mj_jacSubtreeCom` will not raise an error, but the order of elements in the Jacobian will be incorrect. For example:
    ```@example torso
    jac_torso_wrong = zeros(3, model.nv)
    MJ.mj_jacSubtreeCom(model, data, jac_torso_wrong, torso.id)
    @show jac_torso_wrong
    ```

More examples of working with matrices in MuJoCo are provided in [Balancing a Cart-Pole](@ref) and [Humanoid LQR](@ref). For more information on row-major vs column major, see the [Wikipedia page](https://en.wikipedia.org/wiki/Row-_and_column-major_order).

## Wrapped Functions

Similar to the [Python API](https://mujoco.readthedocs.io/en/stable/python.html#functions), we have wrapped several functions to allow the use of native Julia arrays. The majority of functions that take arrays as arguments have been wrapped to apply some basic bounds checks for the size of the arrays, along with a restriction of the type that can be passed in. 

Some of the function signatures have been changed to remove superfluous arguments that can be inferred by the data stored in the Julia arrays, for example:
```@example torso
println(methods(MJ.mju_add).ms[1])
```
vs the original:
```@example torso
println(methods(MJ.LibMuJoCo.mju_add).ms[1])
```

The `mj_` functions that have been wrapped are the defaults and can be found in `src/function_constraints.jl` and are documented in the [MuJoCo API](@ref). If required, the original functions can be accessed inside the submodule `LibMuJoCo`.

## Tips and Tricks

1. This library is a wrapper for MuJoCo. The [documentation for MuJoCo](https://mujoco.readthedocs.io/en/stable/overview.html) is very comprehensive and should be referred to for any queries regarding usage of the simulator.

2. All functions or structs beginning with `mj` directly use the underlying C API.

3. When writing code, keep a REPL open with a sample `data` and `model` object (such as generated in the above examples), and use tab-completition (double tap `tab`) to see what information is available. Check the [MuJoCo docs](https://mujoco.readthedocs.io/en/stable/overview.html) or the [LibMuJoCo Index](@ref) to read the documentation for the parameters.

4. Read documentation for the field names of the objects using the [`show_docs`](@ref) function, e.g. type `show_docs(model, :nv)` or `show_docs(Model, :nv)` in the REPL to get a print out of the docs for that field.


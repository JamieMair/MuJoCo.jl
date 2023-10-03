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
For example, `torso.com` will give you a 3-dimensional vector representing the centre of mass of the torso. 
```@example torso
torso.com
```
All other functions that generate a named wrapper around an object are documented in [Named Access](@ref).

## Row vs. Column-Major Arrays

MuJoCo's underlying C API expects arrays to be **row-major**. However, all Julia arrays are **column-major** by default. When passing arrays to any wrapped C functions, be careful about this distinction. We have plans to provide a nicer API to handle this use case in the future.

All arrays returned by our API are usually of the type `transpose(::UnsafeArray)`, which is how we represent the row-major arrays. This allows you to safely use these arrays alongside any column-major arrays in your Julia code. Keeping this type will also allow for performance specialisation on BLAS operations. 

We demonstrate this below. Suppose we want to compute a Jacobian for the humanoid's torso. There are `model.nv` degrees of freedom in the humanoid, and the torso's centre of mass is a 3-element vector. The Jacobian should therefore be a `3 x nv` array. To ensure the order of elements is correct, we compute it with
```@example torso
jac_torso = zeros(model.nv, 3)
MJ.mj_jacSubtreeCom(model, data, jac_torso, torso.id)
jac_torso = transpose(jac_torso)
```

!!! warning "Beware the row-major order"
    Passing a `3 x nv` array directly to `mj_jacSubtreeCom` will not raise an error. The order of elements in the Jacobian will simply be incorrect. For example:
    ```@example torso
    jac_torso_wrong = zeros(3, model.nv)
    MJ.mj_jacSubtreeCom(model, data, jac_torso_wrong, torso.id)
    jac_torso_wrong
    ```

More examples of working with matrices in MuJoCo are provided in the [Humanoid LQR](@ref) example.

## Tips and Tricks

1. This library is a wrapper for MuJoCo. The [documentation for MuJoCo](https://mujoco.readthedocs.io/en/stable/overview.html) is very comprehensive and should be referred to for any queries regarding usage of the simulator.

2. All functions or structs beginning with `mj_` directly use the underlying C API.

3. When writing code, keep a REPL open with a sample `data` and `model` object (such as generated in the above examples), and use tab-completition (double tap `tab`) to see what information is available. Check the [MuJoCo docs](https://mujoco.readthedocs.io/en/stable/overview.html) or the [LibMuJoCo Index](@ref) to read the documentation for the parameters.


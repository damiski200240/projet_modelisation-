# 3-RRR Parallel Robot: Workspace Optimization

This project aims to optimize the compliant workspace of a 3-RRR parallel robot by tuning its geometric parameters, with the goal of maximizing usable area while ensuring the robot remains compact and free from singularities.

---

## 📌 Objective

**Maximize the area of the compliant workspace**  
under the constraint that the robot's total reach remains within a **30 cm × 30 cm** bounding box.

---

## ⚙️ Robot Description

The 3-RRR robot consists of:
- A triangular fixed base (radius `Rb`)
- Three identical kinematic chains, each with:
  - A revolute joint at the base (motor)
  - Two links (`L1`, `L2`)
  - A moving platform (radius `Re`)

Each leg can operate in two modes: elbow-up or elbow-down, leading to 8 possible working modes.

---

## 🧠 Approach

### 1. **Compliant Workspace Calculation**
- Compute the reachable area of each leg individually (based on joint limits and initial configuration).
- The **compliant workspace** is the intersection of these three reachable zones.
- Implemented using `get_compliant_workspace()`.

### 2. **Objective Function**
We want to **maximize the area** of the compliant workspace.

```python
def objective_function(params):
    Rb, L1, L2, Re = params
    if Rb + 2 * L1 + Re > 300:
        return -1e6  # Penalize configurations that exceed 30 cm reach
    area = get_workspace_area([Rb, L1, L2, Re])
    return -area  # We minimize the negative to maximize area
````

### 3. **Optimization Strategy**

* Use `scipy.optimize.differential_evolution` to find the best parameters.
* Parameters to optimize: `[Rb, L1, L2, Re]`
* Constraint: `Rb + 2 × L1 + Re ≤ 300 mm`

### 4. **Post-Optimization: Joint Limit Validation**

* Once geometric parameters are optimized, calculate the **maximum joint rotation** (`optimize_workspace()`) for a singularity-free operation.
* Input: the determinant map from Jacobian analysis (`det_J_p`)
* Output: maximum joint angle limit per motor in degrees

### 5. **Collision Check in CAD**

* Simulate the optimized design in **SolidWorks**.
* Check for mechanical **interference/collisions** between links at full joint limits.
* If collisions exist, lower the joint limit accordingly.

---

## 🧪 Future Extensions

* Add multi-objective optimization (workspace vs stiffness vs inertia)
* Test with different working modes (`+ + +`, `+ - +`, etc.)
* Visualize singularity maps and workspace overlays in 3D

---

## 📁 Files

* `get_compliant_workspace.py`: Computes compliant workspace
* `optimize_workspace.py`: Finds max joint angle before singularity
* `singularity_loci.py`: Generates determinant maps for Jacobian matrices
* `max_workspace.py`: Contains optimization logic
* `README.md`: This file

---

## 📌 Summary

This pipeline separates **geometric optimization** from **joint angle tuning**, providing a practical and modular approach to parallel robot design.

```


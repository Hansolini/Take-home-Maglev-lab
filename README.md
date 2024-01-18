# Take-home Maglev lab \:bulb\:
The "Take-home Maglev lab" is a small lab in development at the Norwegian University of Science and Technology (NTNU) tailored to students in control and related fields. It is intended as a kit that students can bring home and assemble on their own, with additional lab-assignments for teaching anything from fundamental to advanced concepts within control engineering.

See [the wiki](https://www.ntnu.no/wiki/x/OYSyEw) for more information on the project.

The goal of this lab is to provide something fun and sufficiently complex for students to boost confidence and develop self efficacy in the context of control. 

The development is mainly driven by student contributions in the form of BSc's and MSc's projects, focusing mainly on:
- **Theoretical development:** First principles modeling, control analysis, system configuration analysis
- **Simulator development:** Analyzes and development of efficient MATLAB & Python model implementations
- **Hardware development:** Circuit design and assembly of physical system

This repository contains a
- **[MATLAB simulator](./simulation):** Implementation of several models with examples, see [main.mlx](./simulation/main.mlx) for how to use it,
- **[Design files & software](./physical_system):** PCB files, test code and example usage for the physical system.
- **[Student contributions](./student_contributions):** Code and project reports from student projects.

Following is information on student reports, development history for the hardware and grants provided for the project.
## Project reports \:books\:
### 2022
 1. [M. Brønstad, J. O. Deila, J. Dyerskog, and M. Langklopp, "Magnetic levitation system: Design, prototyping and testing of a digital PID-controller", B.S. thesis, ITK, NTNU, Trondheim, 2022.](./student_contributions/literature/2022_NTNU_bachelor_thesis_MJJM.pdf)
 2. [A. Morselli, "Re-design of a magnetic levitation platform: from an early prototype to a working system", B.S. thesis, DDD, UNIPD, Padova, 2022.](./student_contributions/literature/2022_UNIPD_bachelor_thesis_alberto_morselli.pdf)
 3. [A. Nicetto, "State Space Observers and Controllers for a Take-Home Magnetic Levitation System", B.S. thesis, DDD, UNIPD, Padova, 2022.](./student_contributions/literature/2022_UNIPD_bachelor_thesis_andrea_nicetto.pdf)
 4. [F. D. Marchi, "Modeling and Control of a Magnetic Levitation System", B.S. thesis, DDD, UNIPD, Padova, 2022.](./student_contributions/literature/2022_UNIPD_bachelor_thesis_francesco_de_marchi.pdf)
### 2023
1. [H. A. Engmark, K. T. Hoang, "Modeling and Control of a Magnetic Levitation Platform.", presented at the 22nd IFAC World Congress, Yokohama, 2023 pp. 7276-7281. ](./media/literature/maglev_model_description.pdf)
2. [A. Lincetto, "Towards a Pythonic framework for control and analysis of magnetic levitation systems", B.S. thesis, DDD, UNIPD, Padova, 2023.](./student_contributions/literature/2023_UNIPD_bachelor_thesis_alessandro_lincetto.pdf)
3. [G. Piccolin, "considerations on the software and hardware requirements for the implementation of take-home Maglev control labs", B.S. thesis, DDD, UNIPD, Padova, 2023.](./student_contributions/literature/2023_UNIPD_bachelor_thesis_giulio_piccolin.pdf)
4. [S. Graffer, "State estimation of a Maglev system with the Luenberger observer", B.S. thesis, ITK, NTNU, Trondheim, 2023.](./student_contributions/literature/2023_NTNU_project_thesis_sverre_graffer.pdf)
5. [P. I. D. K. Fosmo, "Control of a magnetic levitation system using feedback linearization", B.S. thesis, ITK, NTNU, Trondheim, 2023.](./student_contributions/literature/2023_NTNU_project_thesis_pål_fosmo.pdf)
## Version history \:rocket\:
| **Version** | **Developer** | **Goal**                                       | **Status**           | **Result**                 |
|-------------|---------------|------------------------------------------------|----------------------|----------------------------|
| V1.0        | M. Brønstad <br> J. O. Deila <br> J. Dyrskog <br> M. Langklopp          | Leviation                                      | \:white_check_mark\: | Stable levitation ~ 2 min  |
| V2.0        | M. Brønstad <br> J. O. Deila <br> J. Dyrskog            | Smaller form factor, more robust               | \:white_check_mark\: | Stable levitation >> 2 min |
| V2.5        | JH Technical  | Fix minor bugs, improve heating and components | \:white_check_mark\: | Same as above              |
| V2.6        | A. Morselli   | Fix faulty wiring, changing sensors    | \:hammer\:             |                            |
| V3.0        | A. Morselli   | Modular redesign for first 'Take-home lab'     | \:hammer\:           |                            |
## Grants \:moneybag\:
| Who | Period | Amount |
|-----|--------|--------|
| Excited NTNU | Spring 2024 | 60kNok | 

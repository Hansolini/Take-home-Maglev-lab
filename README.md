# Take-home Maglev lab \:magnet\:

<table align="center">
  <tr>
    <td align="center">
      <img src="./media/images_and_illustrations/maggy-4.0-physical-setup.png" alt="current hardware" width="400"><br>
      1) Current version of the hardware.
    </td>
    <td align="center">
      <img src="./media/images_and_illustrations/maggy-4.1-electronics-scheme.png" alt="current electronics" width="400"><br>
      2) Overview of the current PCB.
    </td>
    <td align="center">
      <img src="./media/images_and_illustrations/maglev_system_illustration.jpg" alt="magnetic structure" width="400"><br>
      2) Magnetic structure of the system.
    </td>
  </tr>
</table>

The "Take-home Maglev lab" is a small lab in development at the Norwegian University of Science and Technology (NTNU) tailored to students in control and related fields. It is intended as a kit that students can bring home and assemble independently, with additional lab assignments for teaching anything from fundamental to advanced concepts within control engineering.

See [the wiki](https://www.ntnu.no/wiki/x/OYSyEw) for more information on the project, and [this](https://youtube.com/shorts/XnbO15yq0vE) plus [this](https://youtube.com/shorts/F0SiuVEP2tM) videos to see how it looks like.

This repository contains a
- **[MATLAB simulator](./simulation):** Implementation of several models with examples, see [main.mlx](./simulation/main.mlx) for how to use it.
- **[Design files & software](./physical_system):** PCB files, test code, and example usage for the physical system.
- **[Student contributions](./student_contributions):** Code and project reports from student projects.

Following is information on student reports and hardware development history.
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
4. [S. Graffer, "State estimation of a Maglev system with the Luenberger observer", Project thesis, ITK, NTNU, Trondheim, 2023.](./student_contributions/literature/2023_NTNU_project_thesis_sverre_graffer.pdf)
5. [P. I. D. K. Fosmo, "Control of a magnetic levitation system using feedback linearization", Project thesis, ITK, NTNU, Trondheim, 2023.](./student_contributions/literature/2023_NTNU_project_thesis_pål_fosmo.pdf)
6. [S. A. Senkaya, "A take home portable MagLev lab for learning electronics and control", Project thesis, ITK, NTNU, Trondheim, 2023.](./student_contributions/literature/2023_NTNU_project_thesis_sacit_ali_senkaya.pdf)
### 2024
1. [P. I. D. K. Fosmo, S. A. Senkaya, S. Graffer, H. A. Engmark, D. Varagnolo, "Maggy: Hands-on control learning with a maglev system", M.S. thesis, ITK, NTNU, Trondheim, 2024.](https://ntnuopen.ntnu.no/ntnu-xmlui/handle/11250/3153592)
2. [P. I. D. K. Fosmo, H. A. Engmark, D. Varagnolo, "Maglev Systems for Control Education", Maglev24, Malmö, 2024.](https://bth.diva-portal.org/smash/get/diva2:1918096/FULLTEXT01.pdf)
3. [A. Dal Bello, "Tracking Visual Markers For Maglev Platforms", B.S. thesis, DSMN, UNIVE, Venezia, 2024.](./student_contributions/literature/2024_UNIVE_bachelor_thesis_alessandra_dal_bello.pdf)
4. [G. D'Auria, "Computer based state observers for magnet systems", B.S. thesis, DEI, UNIPD, Padova, 2024.](./student_contributions/literature/2024_UNIPD_bachelor_thesis_giuseppe_d'auria.pdf)
### 2025
1. [Ø. Damsgaard, "Moving horizon state estimation for predictive control of magnetic levitation systems", M.S. thesis, ITK, NTNU, Trondheim, 2025](./student_contributions/literature/2025_MSc_thesis_-_Ørnulf_Damsgaard_-_Moving_horizon_state_estimation_for_predictive_control_of_magnetic_levitation_systems.pdf.pdf)
2. [M. Mjelde, "Robust Model Predictive Control for a Magnetic Levitation System", M.S. thesis, ITK, NTNU, Trondheim, 2025](./student_contributions/literature/2025_MSc_thesis_-_Mikael_Mjelde_-_Robust_Model_Predictive_Control_for_a_Maglev.pdf)
3. [L. Ongaro, "Adapting a maglev system for the ball and beam control", B.S. thesis, University of Padova, 2025](./student_contributions/literature/2025_BSc_thesis_Ongaro_Leonardo_Adapting_a_maglev_system_for_the_ball_and_beam_control.pdf)
4. [M. Palamin, "Comunicazione tra un sistema maglev e dispositivi esterni: implementazione e implicazioni per la didattica sulla Teoria del Controllo", B.S. thesis, University of Padova, 2025](./student_contributions/literature/2025_BSc_thesis_Palamin_Manuel_Comunicazione_tra_un_sistema_maglev_e_dispositivi_esterni.pdf)
5. [T. Chinello, "Bode and Nyquist plots for the analysis and control of a magnetic levitation system", B.S. thesis, University of Padova, 2025](./student_contributions/literature/2025_BSc_thesis_Chinello_Tommaso_Bode_and_Nyquist_plots_for_the_analysis_and_control_of_a_magnetic_levitation_system.pdf)
6. [T. A. Jonsson, "Implementation of a Moving Horizon Estimator Using the Acados Framework", M.S. project, ITK, NTNU, Trondheim, 2025](./student_contributions/literature/2025_Prosjektoppgave_-_Thomas_Aleksander_Jonsson_-_MHE_for_Maggy.pdf)
7. [E. Bruaset, "Acados-based Implementation of Nonlinear MPC for Magnetic Levitation", M.S. project, ITK, NTNU, Trondheim, 2025](./student_contributions/literature/2025_Prosjektoppgave_-_Eivind_Bruaset_-_Acados_for_Maggy.pdf)
8. [L. Edvardsen, "Reinforcement Learning for MagLev Control", M.S. project, ITK, NTNU, Trondheim, 2025](./student_contributions/literature/2025_Prosjektoppgave_-_Linus_Edvardsen_-_RL_for_Maggy.pdf)
9. [M. Jullum Faanes, "CasADi-based Implementation of Nonlinear MPC for Magnetic Levitation", M.S. project, ITK, NTNU, Trondheim, 2025](./student_contributions/literature/2025_Prosjektoppgave_-_Marius_Jullum_Faanes_-_SMPC_for_Maggy.pdf)


## Development history \:rocket\:
| **Version** | **Developer**                                                  | **Goal**                                                      | **Development period** |      **Status**      | **Result**                  |
| ----------- | -------------------------------------------------------------- | ------------------------------------------------------------- | ---------------------- | :------------------: | --------------------------- |
| V1.0        | M. Brønstad <br> J. O. Deila <br> J. Dyrskog <br> M. Langklopp | Levitaion                                                     | Spring 2022            | \:white_check_mark\: | Stable levitation ~ 2 min   |
| V2.0        | M. Brønstad <br> J. O. Deila <br> J. Dyrskog                   | Smaller form factor, more robust                              | Summer 2022            | \:white_check_mark\: | Stable levitation >> 2 min  |
| V2.5        | JH Technical AS                                                | Fix minor bugs, improve heating and components                | Spring 2024            | \:white_check_mark\: | Stable levitation >> 2 min  |
| V2.6        | A. Morselli                                                    | Fix faulty wiring, changing sensors                           | Spring 2024            | \:white_check_mark\: | Stable levitation >> 30 min |
| V2.7        | A. Morselli                                                    | Modular design and testbench for V3.0                         | Spring 2024            | \:white_check_mark\: | Decisions on design         |
| V3.0        | A. Morselli                                                    | Modular redesign for first 'Take-home lab'                    | Summer 2024            | \:white_check_mark\: | Decisions on design         |
| V3.1        | M. Leroux                                                      | Adding USB-C power                                            | Fall 2024              | \:white_check_mark\: | Improved power delivery     |
| V4.0        | M. Leroux                                                      | New form factor and additional sensing for improved stability | Fall 2024              | \:white_check_mark\: | Improved control            |
| V4.1        | R. Antonello                                                   | Rerouting PCB tracks and updating components                  | Spring 2025            | \:white_check_mark\: | Meets IPC standards         |
| V4.2        | K. Blom                                                        | Creating a MISO version of the board                          | Spring 2025            | \:white_check_mark\: | MISO board                  |
| V4.3        | A. Morselli                                                    | Improving EM compatibility via some rerouting                 | Fall 2025              | \:white_check_mark\: | Improved EM compatibility   |

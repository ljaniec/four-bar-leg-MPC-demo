# Grafiki generowane

Ten katalog jest wypełniany automatycznie przez pakiet Python podczas budowania prezentacji.

```bash
make -C presentation figures
```

Generowane pliki:

- `leg_three_motor_roles_annotated.png`
- `four_bar_extension_side_view.png`
- `mpc_foot_setpoint_regulation.png`
- `mpc_joint_states.png`
- `mpc_safety_margins.png`

Plik `mpc_foot_setpoint_regulation.png` przedstawia wynik regulacji położenia stopy do jednego stałego punktu zadanego. Linia ruchu stopy jest historią wykonanego ruchu, a nie ścieżką ani trajektorią zadaną.

Nie należy edytować tych plików ręcznie. Nie są wymagane w repozytorium, ponieważ można je odtworzyć z kodu.

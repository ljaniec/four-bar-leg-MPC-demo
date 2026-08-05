# Prezentacja

Ten katalog zawiera polską, 30-slajdową prezentację Beamer o sterowaniu predykcyjnym czworonożnego robota kroczącego z mechanizmem czteroprętowym.

## Budowanie PDF-a

Z katalogu głównego repozytorium:

```bash
./scripts/build_presentation.sh
```

albo bezpośrednio:

```bash
make -C presentation pdf
```

Polecenie najpierw generuje wykresy i wizualizacje z pakietu Pythona, a następnie dwukrotnie uruchamia XeLaTeX. Wynik:

```text
presentation/mpc_dla_robota_czworonoznego.pdf
```

PDF i wygenerowane grafiki są artefaktami budowania i nie muszą być przechowywane w repozytorium.

## Wymagania

- Python z zależnościami projektu;
- XeLaTeX;
- Beamer, TikZ, `fontspec`, `babel-polish`, `listings`, `booktabs` i standardowe pakiety matematyczne.


# 🛠️ Simulateur Robot 3-RRR (Pygame)

## 📌 Description
Ce simulateur permet de visualiser et de contrôler en temps réel un robot parallèle 3-RRR. Il est basé sur Pygame et permet :
- Le déplacement de l’effecteur,
- L’activation de trajectoires prédéfinies,
- L’affichage de la configuration et des diagnostics (det(A), det(B)),
- Le dessin des trajectoires,
- L'exploration des singularités et zones atteignables.

## 🔧 Lancement
Assurez-vous d’avoir installé Pygame :
```bash
pip install pygame
```

Puis lancez simplement :
```bash
python main.py
```

## 🎮 Commandes clavier

### Déplacement (mode clavier)
- `↑ ↓ ← →` : Déplacement dans le plan (x, y)
- `A / E` : Rotation de l’effecteur (angle θ)

### Modes et actions
- `M` : Bascule entre MGI analytique et numérique
- `D` : Active / désactive le dessin de la trajectoire
- `P` : Active / désactive le mode restreint
- `R` : Réinitialise la position du robot à (0, 0, 0)
- `C` : Efface la trajectoire en cours
- `TAB` ou `ESPACE` : Change entre mode clavier et saisie manuelle
- `Entrée` : Valide la valeur saisie dans le mode manuel
- `ESC` : Quitte le simulateur

## 🖱️ Interface graphique

- **RESET** : Réinitialise la position du robot
- **START** : Démarre une trajectoire prédéfinie
- **< / >** : Change la trajectoire sélectionnée

## 📈 Trajectoires disponibles

- `circle` : Cercle
- `square` : Carré
- `line` : Ligne droite
- `heart` : Courbe en cœur
- `eight` : Forme en 8 (stacked ou lemniscate)

## 📋 Modes de contrôle

### Mode clavier
Contrôle direct à l’aide des flèches directionnelles et des touches A / E.

### Mode manuel
Saisie manuelle des coordonnées :
1. `x` (en mètres)
2. `y` (en mètres)
3. `θ` (en degrés)

Une zone verte s'affiche pour chaque champ de saisie.

## 🧠 Diagnostic affiché

- Position actuelle `(x, y, θ)`
- Mode MGI utilisé : `ANALYTIC` ou `NUMERIC`
- Déterminants `det(A)` et `det(B)` (singularités)
- État des modes : `Restreint`, `Dessin`

## ⚠️ Singularités

Le simulateur bloque les mouvements et affiche une erreur si :
- `det(A) ≈ 0` : singularité parallèle
- `det(B) ≈ 0` : singularité série 
- `seuil = 0.0001 ` 


Le robot est alors affiché en rouge.

## 📂 Structure du simulateur 

```
main.py                      # Boucle principale
robot.py                     # Modèle cinématique + affichage 
graphics.py                  # Fonctions de dessin Pygame
controls.py                  # Gestion des événements clavier
trajectoires.py              # Générateurs de trajectoires
fonctions_mathématiques.py   # Outils mathématiques
config.py                    # Paramètres généraux
```

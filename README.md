[![MasterHead](Documentation/GAUL/logo-full.webp)](https://gaulspace.web.app/home)

<h1 align="center">Ordinateur de bord GAUL 2024-2025</h1>

<img align="right" src="https://api.visitorbadge.io/api/visitors?path=https%3A%2F%2Fgithub.com%2FGAULAvionique2024-2025%2FOrdinateur-de-bord&label=Visiteurs&labelColor=%23697689&countColor=%23f47373&style=flat" alt="Visiteurs" />

<p align="left">
  <a href="https://www.facebook.com/gaul.ul" target="_blank">
    <img src="https://raw.githubusercontent.com/rahuldkjain/github-profile-readme-generator/master/src/images/icons/Social/facebook.svg" alt="GAUL Facebook" height="30" width="40" />
  </a>
  <a href="https://www.instagram.com/gaul.ul/" target="_blank">
    <img src="https://raw.githubusercontent.com/rahuldkjain/github-profile-readme-generator/master/src/images/icons/Social/instagram.svg" alt="GAUL Instagram" height="30" width="40" />
  </a>
  <a href="https://www.facebook.com/groupeaerospatialul/" target="_blank">
    <img src="https://raw.githubusercontent.com/rahuldkjain/github-profile-readme-generator/master/src/images/icons/Social/youtube.svg" alt="GAUL Youtube" height="30" width="40" />
  </a>
</p>

## 🌟 **Projet IdeFIX**

IdeFIX est une **balise de repérage embarquée** développée par le GAUL afin de faciliter la récupération et le suivi d’équipements après mission. Le système agit comme une plateforme compacte de télémétrie et de localisation capable de transmettre des informations essentielles en temps réel.

Le projet est conçu pour être :

* Léger et robuste
* Facilement intégrable dans une charge utile ou une fusée
* Optimisé pour une faible consommation énergétique
* Modulable selon les besoins de mission

L’objectif principal d’IdeFIX est d’assurer un repérage fiable des systèmes embarqués pendant les phases de récupération à courte distance.

---

## 📦 **Fonctionnalités Intégrées**

Le système se subdivise en deux cartes qui peuvent inclure plusieurs modules matériels et logiciels :

* **Communication Radio**

  Transmission des données vers une station sol pour le suivi et la récupération.

* **Microcontrôleur principal**

  Assure le traitement des données, la gestion des capteurs et les communications.

* **Gestion d’alimentation**

  Système optimisé pour maximiser l’autonomie et assurer la stabilité électrique.

* **Indicateurs de statut**

  LEDs, buzzer ou autres systèmes de rétroaction pour le diagnostic et le déboggage.

---

## 🛰 **Architecture du Projet**

Le dépôt contient l’ensemble des éléments nécessaires au développement de la balise :

* Firmware embarqué
* Drivers matériels
* Configuration des périphériques
* Documentation technique
* Outils de test et validation
* Scripts utilitaires

L’architecture logicielle vise à simplifier le développement et permettre une maintenance efficace du système.

---

## 📷 **Images du Projet**

Découvrez les différentes itérations et prototypes de la balise de repérage :

[Voir la Showcase](./Documentation/Showcase.md)

<div style="display: flex; justify-content: space-around;">
  <img src="./Documentation/Showcase/PCB.png" alt="PCB IdeFIX" width="45%">
  <img src="./Documentation/Showcase/Prototype.jpg" alt="Prototype IdeFIX" width="45%">
</div>

---

## 📚 **Documentation**

La documentation complète du projet est disponible dans le dossier `Documentation/`.

### Documentation matérielle

* [Composantes](./Documentation/Composantes.md)
* [Schémas](./Documentation/Schemas.md)
* [Assemblage](./Documentation/Assemblage.md)

### Documentation logicielle

* [Architecture logicielle](./Documentation/Architecture.md)
* [Drivers](./Documentation/Drivers.md)
* [Configuration](./Documentation/Configuration.md)
* [Prise en main](./Documentation/Logiciel.md)

### Développement

* [Guide de contribution](./Documentation/Contribution.md)
* [Tests et validation](./Documentation/Validation.md)

---

## ⚙ **Compilation et Déploiement**

Le projet est développé autour de l’écosystème STM32.

### Environnement recommandé

* STM32CubeIDE
* STM32CubeMX
* GCC ARM Toolchain

### Compilation

```bash
git clone https://github.com/GAULAvionique/IdeFIX.git
```

Ouvrir ensuite le projet dans STM32CubeIDE et compiler le firmware selon la configuration cible.

---

## 🛠 **Roadmap**

Fonctionnalités prévues et améliorations futures :

* Optimisation de la consommation énergétique
* Intégration LoRa longue portée
* Système de récupération automatique
* Interface de télémétrie améliorée
* Intégration de nouveaux capteurs
* Outils de visualisation des données

Consultez la [roadmap](./Documentation/Roadmap.md) pour plus de détails.

---

## 💡 **Liens Utiles**

* [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)
* [Documentation STM32](https://www.st.com/)
* [Tutoriels STM32 — DeepBlueEmbedded](https://deepbluembedded.com/stm32-arm-programming-tutorials/)
* [GAUL](https://gaulspace.web.app/home)

---

## 👥 **Auteurs et Contributeurs**

* [@vides119](https://github.com/vides119)
* [@SamLol12](https://github.com/SamLol12)
* [Contributeurs GAUL](./Documentation/Participants.md)

---

🏠 Retour au la [page d'accueil](https://github.com/GAULAvionique2024-2025)

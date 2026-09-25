// Compile: typst compile main.typ report.pdf

#let report-title = "TP n°3 : ICP"
#let student-name = "Yaqi ZHANG"
#let report-date = datetime.today().display("[day] [month repr:long] [year]")

#set document(title: report-title, author: student-name)

#set page(
  paper: "a4",
  margin: (top: 2.4cm, bottom: 2.4cm, x: 2.3cm),
  numbering: "1",
)

#set text(font: "Libertinus Serif", size: 11pt, lang: "fr")
#set par(justify: true, leading: 0.65em, first-line-indent: 1.2em)
#set heading(numbering: "1.")
#set figure(supplement: [Figure])

// Helper for a single figure.
#let report-figure(path, caption, width: 90%) = figure(
  image(path, width: width),
  caption: caption,
)

// Helper for multiple equally high images, each with its own small subtitle.
// `paths` and `subtitles` must have the same number of elements.
#let report-image-row(paths, subtitles, height: 4cm, caption: none) = figure(
  grid(
    columns: paths.len(),
    gutter: 0.45em,
    ..range(0, paths.len()).map(index => align(center)[
      #stack(
        dir: ttb,
        spacing: 0.4em,
        image(paths.at(index), height: height),
        text(size: 9pt)[#subtitles.at(index)],
      )
    ]),
  ),
  caption: caption,
)

// First page header: title, name and date, without a separate cover page.
#align(center)[
  #v(1.3cm)
  #text(size: 17pt, weight: "bold")[#report-title]

  #v(0.7cm)
  #student-name

  #v(0.35cm)
  #report-date
]



#v(0.5cm)

#link("https://github.com/yaqinoel/HAI917I-IG_Avance_TPs/tree/6f53aea15a36372551602d4d5895613d9d5e0cf5/TP3_Recalage")[Line du code sur github (Cliquez pour visiter)]

#v(0.5cm)

= ACP

Les axes principaux visualisent l'orientation globale des deux nuages.

#report-image-row(
  (
    "figures/acp.png",
  ),
  (
    [Axes principaux des deux nuages],
  ),
  height: 6cm,
  caption: [ACP appliquée aux deux nuages de points.],
)

= SVD

Les correspondances des points sont connues, le recalage par SVD superpose presque exactement les deux nuages, y compris dans les détails.

#report-image-row(
  (
    "figures/svd_init.png",
    "figures/svd_recalage.png",
    "figures/svd_detail.png",
  ),
  (
    [Avant recalage],
    [Après recalage],
    [Détail du recouvrement],
  ),
  height: 6cm,
  caption: [Recalage par SVD avec correspondances connues.],
)

= ICP

== Test ICP

Un premier essai avec une ACP classique suivie de correspondances par plus proche voisin donne un mauvais recalage. L'ambiguïté du sens des axes peut créer de fausses correspondances, puis l'ICP converge vers un minimum local.

#report-figure(
  "figures/icp_nearest_neighbor_failure.png",
  [Exemple de recalage incorrect avec des correspondances au plus proche voisin.],
  width: 75%,
)

Pour améliorer l'initialisation, les quatre orientations valides des axes ACP sont comparées. Celle qui minimise la distance aux plus proches voisins est retenue avant l'ICP.

Sur les nuages de dinosaure fortement sous-échantillonnés, l'ICP avec projection HPSS rapproche les deux formes.

#report-image-row(
  (
    "figures/icp_hpss_intial.png",
    "figures/icp_hpss_result.png",
  ),
  (
    [Avant recalage],
    [Après recalage],
  ),
  height: 6cm,
  caption: [ICP avec projection des correspondances sur la surface HPSS.],
)

La projection HPSS réduit la dépendance aux points échantillonnés, mais peut attirer un point vers une mauvaise région si le recouvrement est faible.

== Recalage de pointssets identiques à une transformation rigide près

Après une rotation et une translation aléatoires, les deux copies du même nuage se superposent presque parfaitement.

#report-image-row(
  (
    "figures/icp_rigid_transform.png",
    "figures/icp_rigid_transform_detial.png",
  ),
  (
    [Vue générale],
    [Détail du recouvrement],
  ),
  height: 6cm,
  caption: [ICP entre deux exemplaires du même nuage.],
)

== Recalage de pointssets incomplets

Le nuage complet et le nuage partiel se rapprochent, mais certaines régions restent décalées. Les points absents du nuage partiel n'ont pas de correspondance fiable et perturbent le recalage.

#report-image-row(
  (
    "figures/icp_incomplets_intial.png",
    "figures/icp_incomplets_result.png",
  ),
  (
    [Avant recalage],
    [Après recalage],
  ),
  height: 6cm,
  caption: [Recalage d'un nuage complet avec un nuage partiel.],
)

== Recalage de pointssets inconsistents

Le nuage partiel se rapproche du nuage complet, mais leur zone commune ne se superpose pas : le recalage échoue. Une piste serait de tenir compte des caractéristiques locales, notamment de la cohérence des normales, lors de la recherche des correspondances.

#report-image-row(
  (
    "figures/icp_inconsistents_initial.png",
    "figures/icp_inconsistents_result.png",
  ),
  (
    [Avant recalage],
    [Après recalage],
  ),
  height: 6cm,
  caption: [Échec du recalage du nuage partiel vers le nuage complet.],
)

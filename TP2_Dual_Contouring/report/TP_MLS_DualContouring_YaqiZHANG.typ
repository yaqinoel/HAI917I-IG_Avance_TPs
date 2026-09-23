// Compile from this directory: typst compile TP_MLS_DualContouring_YaqiZHANG.typ

#let report-title = "TP n°1 et n°2 : MLS et Dual Contouring"
#let student-name = "Yaqi ZHANG"
#let report-date = "23 septembre 2026"

#set document(title: report-title, author: student-name)

#set page(
  paper: "a4",
  margin: (top: 1.8cm, bottom: 2cm, x: 2.3cm),
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
  #text(size: 15pt, weight: "bold")[#report-title]
  #v(0.2cm)
  #text(size: 10pt)[#student-name · #report-date]
  #v(0.3cm)
  #text(size: 9pt)[
    #link("https://github.com/yaqinoel/HAI917I-IG_Avance_TPs/tree/968a5a67f69edc1f7d3359b33305b603e2b007cc/TP1_HPSS")[Code source TP1 - HPSS]
    #h(1.2em)
    #link("https://github.com/yaqinoel/HAI917I-IG_Avance_TPs/tree/968a5a67f69edc1f7d3359b33305b603e2b007cc/TP2_Dual_Contouring")[Code source TP2 - Dual Contouring]
  ]
]

#v(0.4cm)

= HPSS

== Projection de points répartis aléatoirement

Les 1 000 points de départ, répartis dans le cube $[-4, 4]^3$, entourent le nuage de points du lapin.

#report-image-row(
  (
    "figures/points_random.png",
  ),
  (
    [Points initiaux (rose) et nuage de référence (bleu).],
  ),
  height: 6cm,
  caption: [Répartition des points avant la projection HPSS.],
)

== Influence de la taille du noyau gaussien

Avec un noyau de 0,5, les points éloignés reçoivent des poids presque nuls : certains ne bougent donc pas. À 0,8 et 1,0, les points isolés sont beaucoup moins nombreux et les résultats sont proches.

#report-image-row(
  (
    "figures/gaussien_noyau_0_5.png",
    "figures/gaussien_noyau_0_8.png",
    "figures/gaussien_noyau_1_0.png",
  ),
  (
    [Taille 0,5],
    [Taille 0,8],
    [Taille 1,0],
  ),
  height: 6cm,
  caption: [Points projetés (verts) pour trois tailles du noyau gaussien ; nuage de référence en bleu.],
)

== Influence du nombre d'itérations

Après 2 itérations, certains points restent à l'écart du lapin. Le résultat s'améliore à 5 itérations, puis change peu entre 10 et 20.

#report-image-row(
  (
    "figures/iteration_2.png",
    "figures/iteration_5.png",
    "figures/iteration_10.png",
    "figures/iteration_20.png",
  ),
  (
    [2 itérations],
    [5 itérations],
    [10 itérations],
    [20 itérations],
  ),
  height: 4.2cm,
  caption: [Points projetés (verts) selon le nombre d'itérations ; nuage de référence en bleu.],
)

= Dual Contouring

== Résultat du dual contouring

La silhouette du lapin est reconnaissable, mais le maillage obtenu avec une grille de $32^3$ reste anguleux.

#report-image-row(
  (
    "figures/dual_contouring.png",
  ),
  (
    [Sommets au centre des cellules.],
  ),
  height: 6cm,
  caption: [Reconstruction par dual contouring sur une grille de $32^3$.],
)

== Position du sommet dans la cellule

La projection du centre par HPSS réduit l'aspect en blocs et rapproche le maillage du nuage de points, malgré quelques irrégularités locales.

#report-image-row(
  (
    "figures/dual_contouring.png",
    "figures/dual_contouring_hpss_center.png",
  ),
  (
    [Centre de la cellule],
    [Centre projeté par HPSS],
  ),
  height: 6cm,
  caption: [Comparaison des deux positions de sommet avec une grille de $32^3$.],
)

== Influence de la résolution de la grille

De $32^3$ à $128^3$, les contours du lapin deviennent plus fins et les sommets suivent mieux les détails ; leur nombre augmente nettement.

#report-image-row(
  (
    "figures/dual_contouring_hpss_center.png",
    "figures/dual_contouring_hpss_64.png",
    "figures/dual_contouring_hpss_128.png",
  ),
  (
    [Grille de $32^3$],
    [Grille de $64^3$],
    [Grille de $128^3$],
  ),
  height: 6cm,
  caption: [Reconstruction avec trois résolutions de grille.],
)

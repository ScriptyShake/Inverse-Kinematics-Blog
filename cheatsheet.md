---
layout: default
title: "⬇️ Markdown cheatsheet"
---

# 
For a more comprehensive, github-flavored guide check: https://docs.github.com/en/get-started/writing-on-github. 

## Table of content
- [Headings](#headings)
- [Emphasis](#emphasis)
- [Lists](#lists)
- [Links](#links)
- [Code](#code)
- [Mathematical Expressions](#mathematical-expressions)
- [Images](#images)
- [Videos](#videos)

## Headings

# Heading 1
## Heading 2
### Heading 3
#### Heading 4
##### Heading 5

```
# Heading 1
## Heading 2
### Heading 3
#### Heading 4
##### Heading 5
```

## Emphasis

Bold
**The quick brown fox jumps over the lazy dog.**

Italic
*The quick brown fox jumps over the lazy dog.*

Strike-through
~~The quick brown fox jumps over the lazy dog.~~

```markdown
Bold
**The quick brown fox jumps over the lazy dog.**

Italic
*The quick brown fox jumps over the lazy dog.*

Strike-through
~~The quick brown fox jumps over the lazy dog.~~
```

## Lists
Ordered lists require manual numbering, while unhordered lists can be done with multiple characters (but the recommended one is `-`)

1. First ordered list item
2. Another item
    - Unordered sub-list. 
1. Actual numbers don't matter, just that it's a number
    1. Ordered sub-list
4. And another item.


```markdown
1. First ordered list item
2. Another item
    - Unordered sub-list. 
1. Actual numbers don't matter, just that it's a number
    1. Ordered sub-list
4. And another item.
```

## Links
[Inline-style link](https://www.google.com)

[Relative link to a file in the repository](../evidence/ILO01.md), use `..` to access parent folders 

[Relative link to an header in the current document](#headings)

[Relative link to an header in another document](evidence/README.md#what-is-in-this-template)

```markdown
[Inline-style link](https://www.google.com)

[Relative link to a file in the repository](../evidence/ILO01.md)

[Relative link to an header in the current document](#headings)

[Relative link to an header in another document](evidence/README.md#what-is-in-this-template)

```

Note that it is also possible to use references and footnotes, for that check 

## Code 
Code can be `inlined` 

Like this: `auto entity = Engine.ECS().CreateEntity();`

```markdown
`auto entity = Engine.ECS().CreateEntity();`
```

Or it can be put in a code block (optionally you can enable syntax highlight by providing a language name after the opening ` ``` `)

```cpp
struct Transform{
    vec3 stuff;
}
```

````markdown
```cpp
struct Transform{
    vec3 stuff;
}
```
````

You can also reference code from your repo (can reference old commits) see:
https://docs.github.com/en/get-started/writing-on-github/working-with-advanced-formatting/creating-a-permanent-link-to-a-code-snippet

## Mathematical Expressions

Mathematical equations in markdown are based uses `MathJax` which in turn uses `LaTeX` as markup. Check http://tug.ctan.org/info/undergradmath/undergradmath.pdf for a small cheatsheet of `LaTeX` for mathematical expressions.

Expressions can be inlined $\Delta x = \vec{v} \Delta t$
```markdown
$\Delta x = \vec{v} \Delta t$
```

or presented as a block

$$
L_o(\omega_o) = L_e(\omega_o) + \int_{\Omega} f_r(\omega_o, \omega_i) \cdot L_i(\omega_i) \cdot \cos\theta_{i} \cdot d\omega_i
$$

```markdown
$$
L_o(\omega_o) = L_e(\omega_o) + \int_{\Omega} f_r(\omega_o, \omega_i) \cdot L_i(\omega_i) \cdot \cos\theta_{i} \cdot d\omega_i
$$
```

## Images 
![alt text](assets/media/download_github.png)

```markdown
![alt text](assets/media/download_github.png)
```

## Videos
You can embed YouTube videos by clicking `share` and then `embed` in YouTube.

You can also embed videos uploaded to the repository.

*__Warning:__ Videos might not display correctly on GitHub or in your own markdown editor. Make sure that they at least display correctly in the website release!*

<video width="320" height="240" controls>
  <source src="assets/media/sample_video.mp4" type="video/mp4">
</video>

```markdown
<video width="320" height="240" controls>
  <source src="assets/media/sample_video.mp4" type="video/mp4">
</video>
```

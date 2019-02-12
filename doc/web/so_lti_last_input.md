---
title: Homere Control
layout: default
---

<script src="https://cdn.mathjax.org/mathjax/latest/MathJax.js?config=TeX-AMS-MML_HTMLorMML" type="text/javascript"></script>
<script type="text/x-mathjax-config">
MathJax.Hub.Config({
  TeX: { equationNumbers: { autoNumber: "AMS" } }
});
</script>


# Finding last input


We consider a second order LTI system described by equation

$$
\begin{equation}
y_{k+1} = a_1 u_k + a_0 u_{k-1} - b_1 y_k -b_0 y_{k-1}
\label{eq:plant}
\end{equation}
$$

The problem at stake is, given a time serie of outputs $$y_k, y_{k-1}, \dots, y_{k-j}$$, find the last used input, $$u_{k-1}$$


As a numerical example, for a (asymptotically stable) plant with natural frequency $$\omega=1rad/s$$ and damping $$\xi=0.5$$, sampled at 100Hz, we get

$$
\begin{equation}
4.983 e^{-5} u_k + 4.967e^{-5} u_{k-1} = y_{k+1} - 1.99 y_k + 0.99 y_{k-1}
\end{equation}
$$


## Matrix formulation

At times k, k-1, ..., k-j  equation \eqref{eq:plant} can be rewritten

$$
\begin{align*}
a_1 u_{k-1} + a_0 u_{k-2} & =  y_{k} + b_1 y_{k-1} + b_0 y_{k-2}   \\
a_1 u_{k-2} + a_0 u_{k-3} & =  y_{k-1} + b_1 y_{k-2} + b_0 y_{k-3} \\
\vdots & \\
a_1 u_{k-j} + a_0 u_{k-j-1} & =  y_{k-j+1} + b_1 y_{k-j} + b_0 y_{k-j-1}
\label{eq:plantk}
\end{align*}
$$

which can be matricially summarized as

$$
\begin{pmatrix}
a_1 & a_0 & 0 & 0   \\
0   & a_1 & a_0 & 0 \\
0 & 0 & a_1 & a_0   \\
\end{pmatrix}
\begin{pmatrix}u_{k-1} \\ u_{k-2} \\ u_{k-3} \\ u_{k-4} \end{pmatrix} =
\begin{pmatrix}
1 & b_1 & b_0 & 0 & 0    \\
0 & 1 & b_1 & b_0 & 0    \\
0 & 0 & 1 & b_1 & b_0
\end{pmatrix}
\begin{pmatrix}y_{k} \\ y_{k-1} \\ y_{k-2} \\ y_{k-3} \\ y_{k-4} \end{pmatrix}
$$

It becomes apparent that our problem is an underdetermined linear system.



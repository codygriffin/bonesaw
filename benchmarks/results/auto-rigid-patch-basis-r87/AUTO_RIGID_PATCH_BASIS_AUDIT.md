# Bonesaw automatic rigid-patch basis · r87

## Outcome

**Admission: PASS.** Construction derives a deterministic six-row rigid-body basis from fixed same-frame contact geometry. The selected per-point modes enter the existing descriptor fingerprint; execution remains the zero-allocation R85 path. Python and Pinocchio independently rebuild rank, hard dynamics/contact, the implied full sole manifold, hull, weighted CoP, and support margin. No policy or physics rollout is used.

## Compiler-selected basis

['| signal | result |', '|---|---|', '| selected modes | [3, 0, 2, 1, 1] · Disabled=0 Locked=1 Normal=2 Rolling=3 |', '| rows / independent rank | 6 / [6] |', '| minimum singular value | 0.048081 |', '| implied all-point acceleration L∞ | 4.665e-13 |', '| explicit R85 modes / σmin | [1, 2, 0, 3, 0] / 0.048023 |', '| repeat descriptor / differs from all locked | True / True |']

The compiler enumerates only prefix-compatible Disabled/Normal/Rolling/Locked declarations totaling six scalar rows, rejects deficient candidates, and selects the candidate with the largest minimum singular value. Point coordinates are centered before selection, so a body-frame origin translation cannot change the choice.

## Independent physical oracle

['| signal | result |', '|---|---|', '| Pinocchio dynamics L∞ | 1.358e-10 |', '| Pinocchio selected contact L∞ | 2.691e-13 |', '| independent minimum CoP margin | 0.020000000 m |', '| reported-vs-independent margin delta | 2.429e-17 m |', '| allocation calls / bytes | 0 / 0 |']

## Invalid geometry

['| case | result |', '|---|---|', '| four collinear same-frame points | rigid patch 8401 point geometry is non-finite or rank deficient |', '| typed rejection | True |']

## Solve timing (60 untrimmed calls/profile)

['| agents | auto p50 µs | explicit R85 p50 µs | all locked p50 µs | auto vs explicit | auto vs locked |', '|---|---|---|---|---|---|', '| 1 | 1671.136 | 1251.549 | 1814.001 | +33.53% | -7.88% |', '| 8 | 8903.712 | 9701.635 | 13075.555 | -8.22% | -31.91% |', '| 32 | 36741.372 | 39108.467 | 54184.931 | -6.05% | -32.19% |']

Construction-time enumeration is excluded from solve timing. Automatic and explicit bases can choose different legal lower-layer qdd/force optima, so this gate claims the same independently checked rigid-foot hard manifold—not bitwise cross-basis solutions.

## Deliberate boundary

R87 assumes one body frame and canonical frame X/Y/Z matching runtime tangent-X/tangent-Y/normal inputs. General authored contact frames, state-dependent switching, RollingWheel constants, CoM-in-polygon tasks, live finite-foot composition, and CUDA remain unavailable.

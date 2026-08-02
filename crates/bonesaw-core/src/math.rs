use nalgebra::{
    Isometry3, Quaternion, SMatrix, SVector, Translation3, UnitQuaternion, Vector2, Vector3,
};
use serde::{Deserialize, Serialize};

pub type Vec2 = Vector2<f64>;
pub type Vec3 = Vector3<f64>;
pub type Transform3 = Isometry3<f64>;
pub type ControlTime = i64;
pub type SymmetricMat6 = SMatrix<f64, 6, 6>;

/// Bit-preserving serde adapter for an isometry.
///
/// `nalgebra` deliberately normalizes a `UnitQuaternion` while deserializing
/// it. That is a sensible general-purpose default, but the extra normalization
/// can move an already-unit component by one ULP and therefore invalidate a
/// content-addressed MotionProgram after an otherwise lossless archive round
/// trip. The compiler has already constructed and validated the unit
/// quaternion, so archives preserve its four components exactly and reject
/// malformed rotations instead of silently changing them.
pub(crate) mod serde_transform3 {
    use serde::{
        Deserialize, Deserializer, Serialize, Serializer,
        de::{Error, Unexpected},
    };

    use super::{Quaternion, Transform3, Translation3, UnitQuaternion};

    #[derive(Serialize)]
    struct TransformArchive {
        rotation: [f64; 4],
        translation: [f64; 3],
    }

    #[derive(Deserialize)]
    struct TransformArchiveOwned {
        rotation: [f64; 4],
        translation: [f64; 3],
    }

    pub fn serialize<S>(transform: &Transform3, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let quaternion = transform.rotation.quaternion();
        TransformArchive {
            rotation: [quaternion.i, quaternion.j, quaternion.k, quaternion.w],
            translation: [
                transform.translation.vector.x,
                transform.translation.vector.y,
                transform.translation.vector.z,
            ],
        }
        .serialize(serializer)
    }

    pub fn deserialize<'de, D>(deserializer: D) -> Result<Transform3, D::Error>
    where
        D: Deserializer<'de>,
    {
        let archived = TransformArchiveOwned::deserialize(deserializer)?;
        if !archived
            .rotation
            .iter()
            .chain(&archived.translation)
            .all(|value| value.is_finite())
        {
            return Err(D::Error::invalid_value(
                Unexpected::Other("non-finite transform component"),
                &"a finite rigid transform",
            ));
        }
        let [i, j, k, w] = archived.rotation;
        let norm_squared = i.mul_add(i, j.mul_add(j, k.mul_add(k, w * w)));
        if (norm_squared - 1.0).abs() > 1e-12 {
            return Err(D::Error::invalid_value(
                Unexpected::Float(norm_squared),
                &"a unit quaternion with squared norm within 1e-12 of one",
            ));
        }
        let [x, y, z] = archived.translation;
        Ok(Transform3 {
            translation: Translation3::new(x, y, z),
            rotation: UnitQuaternion::new_unchecked(Quaternion::new(w, i, j, k)),
        })
    }
}

/// Featherstone motion vector ordered `[angular; linear]`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(transparent)]
pub struct Motion6(pub SVector<f64, 6>);

/// Featherstone force vector ordered `[moment; force]`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(transparent)]
pub struct Force6(pub SVector<f64, 6>);

/// Spatial acceleration ordered `[angular; linear]`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(transparent)]
pub struct SpatialAcceleration6(pub SVector<f64, 6>);

pub(crate) fn parse_vec3(value: &str) -> Option<Vec3> {
    let mut fields = value.split_ascii_whitespace().map(str::parse::<f64>);
    let x = fields.next()?.ok()?;
    let y = fields.next()?.ok()?;
    let z = fields.next()?.ok()?;
    if fields.next().is_some() || !x.is_finite() || !y.is_finite() || !z.is_finite() {
        return None;
    }
    Some(Vec3::new(x, y, z))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Deserialize)]
    struct ArchivedTransform {
        #[serde(with = "serde_transform3")]
        value: Transform3,
    }

    #[test]
    fn transform_archive_preserves_quaternion_components_bitwise() {
        let expected = 0.9961786849744957_f64;
        let archived: ArchivedTransform = serde_json::from_str(
            r#"{"value":{"rotation":[0.0,-0.08733858026373247,0.0,0.9961786849744957],"translation":[0.0,0.052,-0.030465]}}"#,
        )
        .unwrap();
        assert_eq!(
            archived.value.rotation.quaternion().w.to_bits(),
            expected.to_bits()
        );
    }
}

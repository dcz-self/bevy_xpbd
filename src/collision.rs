//! Collision events, contact data and helpers.

use crate::prelude::*;

/// An event that is sent for each contact pair during the narrow phase.
#[derive(Clone, Debug, PartialEq)]
pub struct Collision(pub Contact);

/// An event that is sent when two entities start colliding.
#[derive(Clone, Debug, PartialEq)]
pub struct CollisionStarted(pub Entity, pub Entity);

/// An event that is sent when two entities stop colliding.
#[derive(Clone, Debug, PartialEq)]
pub struct CollisionEnded(pub Entity, pub Entity);

/// Data related to a contact between two bodies.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Contact {
    /// First entity in the contact.
    pub entity1: Entity,
    /// Second entity in the contact.
    pub entity2: Entity,
    /// Local contact point 1 in local coordinates.
    pub local_r1: Vector,
    /// Local contact point 2 in local coordinates.
    pub local_r2: Vector,
    /// Local contact point 1 in world coordinates.
    pub world_r1: Vector,
    /// Local contact point 2 in world coordinates.
    pub world_r2: Vector,
    /// Contact normal from contact point 1 to 2.
    pub normal: Vector,
    /// Penetration depth.
    pub penetration: Scalar,
}

/// Computes one pair of contact points between two shapes.
#[allow(clippy::too_many_arguments)]
pub(crate) fn compute_contact(
    ent1: Entity,
    ent2: Entity,
    pos1: Vector,
    pos2: Vector,
    local_com1: Vector,
    local_com2: Vector,
    rot1: &Rot,
    rot2: &Rot,
    shape1: &Shape,
    shape2: &Shape,
) -> Option<Contact> {
    if let Ok(Some(contact)) = parry::query::contact(
        &utils::make_isometry(pos1, rot1),
        shape1.0.as_ref(),
        &utils::make_isometry(pos2, rot2),
        shape2.0.as_ref(),
        0.0,
    ) {
        let world_r1 = Vector::from(contact.point1) - pos1 + local_com1;
        let world_r2 = Vector::from(contact.point2) - pos2 + local_com2;

        return Some(Contact {
            entity1: ent1,
            entity2: ent2,
            local_r1: rot1.inv().rotate(world_r1),
            local_r2: rot2.inv().rotate(world_r2),
            world_r1,
            world_r2,
            normal: contact.normal1.into(),
            penetration: -contact.dist,
        });
    }
    None
}

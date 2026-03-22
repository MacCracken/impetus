//! Error types for impetus.

use thiserror::Error;

#[derive(Debug, Error, PartialEq)]
pub enum ImpetusError {
    #[error("body not found: {0}")]
    BodyNotFound(String),

    #[error("collider not found: {0}")]
    ColliderNotFound(String),

    #[error("joint not found: {0}")]
    JointNotFound(String),

    #[error("invalid configuration: {0}")]
    InvalidConfig(String),

    #[error("serialization error: {0}")]
    Serialize(String),

    #[error("deserialization error: {0}")]
    Deserialize(String),

    #[error("physics error: {0}")]
    Physics(String),
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn error_display() {
        let err = ImpetusError::BodyNotFound("handle-42".into());
        assert_eq!(err.to_string(), "body not found: handle-42");
    }

    #[test]
    fn error_variants() {
        let errors = vec![
            ImpetusError::BodyNotFound("a".into()),
            ImpetusError::ColliderNotFound("b".into()),
            ImpetusError::JointNotFound("c".into()),
            ImpetusError::InvalidConfig("d".into()),
            ImpetusError::Serialize("e".into()),
            ImpetusError::Deserialize("f".into()),
            ImpetusError::Physics("g".into()),
        ];
        for err in &errors {
            assert!(!err.to_string().is_empty());
        }
    }

    #[test]
    fn error_equality() {
        assert_eq!(
            ImpetusError::Physics("boom".into()),
            ImpetusError::Physics("boom".into())
        );
        assert_ne!(
            ImpetusError::Physics("a".into()),
            ImpetusError::BodyNotFound("a".into())
        );
    }
}

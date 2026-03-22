use thiserror::Error;

#[derive(Debug, Error)]
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

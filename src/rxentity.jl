using Rocket
export create_entity

mutable struct EntityActor{T} <: Rocket.Actor{Any}
    entity::T
    subscription::Union{Nothing,Rocket.Teardown}
end

entity(actor::EntityActor) = actor.entity

"""
    Rocket.on_next!(actor::EntityActor{ActiveEntity}, observation)})

Handles the logic for an incoming observation for an active entity. This means that the entity will update its state, incorporate the observation
into its state, and then send an action to all of its subscribers. The action is determined by the `what_to_send` function. Emissions can be filtered
by implementing the `emits` function.

This function is automatically called whenever the entity receives an observation on it's sensor. The `observation` will contain the data sent by the
emitter as well as a reference to the emitter itself. 
"""
function Rocket.on_next!(
    actor::EntityActor{E} where {E<:AbstractEntity{T,S,<:ActiveEntity} where {T,S}},
    observation,
)
    subject = entity(actor)
    update!(subject)
    receive!(subject, observation)
    foreach(subscribers(subject)) do listener
        if emits(subject, listener, observation)
            action = what_to_send(listener, subject, observation)
            __send!(listener, subject, action)
        end
    end
end

"""
    Rocket.on_next!(actor::EntityActor{PassiveEntity}, observation)

Handles the logic for an incoming observation for a passive entity. This means that we will only incorporate the observation into the entity's state.
"""
function Rocket.on_next!(
    actor::EntityActor{E} where {E<:AbstractEntity{T,S,<:PassiveEntity} where {T,S}},
    observation,
)
    receive!(entity(actor), observation)
end

function terminate!(actor::EntityActor)
    teardown = actor.subscription
    unsubscribe!(teardown)
end

mutable struct EntityProperties{S,E}
    state_space::S
    is_environment::E
    terminated::Terminated
    clock::Clock
    timer::Union{Timer,Nothing}
    entity_actor::Union{EntityActor,Nothing}
end

EntityProperties(state_space::DiscreteEntity, is_environment; real_time_factor=1.0) =
    EntityProperties(
        state_space,
        is_environment,
        Terminated(false),
        ManualClock(),
        nothing,
        nothing,
    )
EntityProperties(
    state_space::ContinuousEntity,
    is_environment;
    real_time_factor::Real=1.0,
) = EntityProperties(
    state_space,
    is_environment,
    Terminated(false),
    WallClock(real_time_factor),
    nothing,
    nothing,
)

function terminate!(properties::EntityProperties)
    terminate!(properties.timer)
    terminate!(properties.entity_actor)
end

"""
    RxEntity

The RxEntity is the vanilla implementation of an `AbstractEntity` that is used in most cases. It is a wrapper around an entity that adds the following functionality:

- A `MarkovBlanket` that contains the actuators and sensors of the entity
- A `EntityProperties` that contains the state space, whether or not the entity is active, and the real time factor
- A `EntityActor` that handles the logic for receiving observations and sending actions
"""
struct RxEntity{T,S,E,A} <: AbstractEntity{T,S,E}
    decorated::T
    markov_blanket::MarkovBlanket{S,A}
    properties::EntityProperties{S,E}
end

"""
    create_entity(entity; is_discrete = false, is_active = false, real_time_factor = 1)

Creates an `RxEntity` that decorates a given entity. The `is_discrete` and `is_active` parameters determine whether or not the entity lives in a discrete or continuous state-space,
and whether or not the entity is active or passive. The `real_time_factor` parameter determines how fast the entity's clock ticks. This function can be used as a lower-level API
in order to create a more complex network of entities.
"""
function create_entity(
    entity;
    is_discrete::Bool=false,
    is_active::Bool=false,
    real_time_factor=1,
)
    state_space = is_discrete ? DiscreteEntity() : ContinuousEntity()
    operation_type = is_active ? ActiveEntity() : PassiveEntity()
    return create_entity(entity, state_space, operation_type, real_time_factor)
end

function create_entity(entity, state_space, active_or_passive, real_time_factor::Real=1)
    result = RxEntity(
        entity,
        MarkovBlanket(state_space, action_type(entity)),
        EntityProperties(
            state_space,
            active_or_passive;
            real_time_factor=real_time_factor,
        ),
    )
    entity_actor = EntityActor(result, nothing)
    entity_actor.subscription = subscribe!(observations(result), entity_actor)
    properties(result).entity_actor = entity_actor
    return result
end

function Base.:(==)(a::RxEntity, b::RxEntity)
    return a === b
end

# Entities are unique wrappers with their own actuator/sensor sets. Equality and
# hashing must be identity-based so that two distinct entities (even ones wrapping
# `==`-equal decorated objects) are never conflated in `in`/`∈`/Dict-keyed routing.
Base.isequal(a::RxEntity, b::RxEntity) = a === b
Base.hash(a::RxEntity, h::UInt) = hash(objectid(a), h)

function Base.show(io::IO, entity::RxEntity{T,ContinuousEntity,E} where {T,E})
    print(io, "Continuous RxEntity{", typeof(decorated(entity)), "}")
end

function Base.show(io::IO, entity::RxEntity{T,DiscreteEntity,E} where {T,E})
    print(io, "Discrete RxEntity{", typeof(decorated(entity)), "}")
end

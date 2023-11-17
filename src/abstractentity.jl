using Rocket

struct IsEnvironment end
struct IsNotEnvironment end

struct DiscreteEntity end
struct ContinuousEntity end

export AbstractEntity,
    add!,
    send!,
    update!,
    receive!,
    is_subscribed,
    subscribers,
    subscribed_to,
    terminate!,
    is_terminated,
    animate_state,
    subscribe_to_observations!

"""
    AbstractEntity{T}

The AbstractEntity type supertypes all entities. It describes basic functionality all entities should have. It is assumed that every 
entity has a markov blanket, which has actuators and sensors. The AbstractEntity also has a field that describes whether or not the
entity is terminated. 
"""
abstract type AbstractEntity{T,S,E} end

decorated(entity::AbstractEntity) = entity.decorated
markov_blanket(entity::AbstractEntity) = entity.markov_blanket
properties(entity::AbstractEntity) = entity.properties
is_terminated(entity::AbstractEntity) = is_terminated(properties(entity).terminated)
state_space(entity::AbstractEntity) = properties(entity).state_space
is_environment(entity::AbstractEntity{T,S,IsEnvironment} where {T,S}) = true
is_environment(entity::AbstractEntity{T,S,IsNotEnvironment} where {T,S}) = false
clock(entity::AbstractEntity) = properties(entity).clock
Base.time(entity::AbstractEntity) = time(clock(entity))

last_update(entity::AbstractEntity{T,ContinuousEntity,E}) where {T,E} =
    last_update(clock(entity))

observations(entity::AbstractEntity) = observations(markov_blanket(entity))
actuators(entity::AbstractEntity) = actuators(markov_blanket(entity))
sensors(entity::AbstractEntity) = sensors(markov_blanket(entity))
subscribers(entity::AbstractEntity) = collect(keys(actuators(entity)))
subscribed_to(entity::AbstractEntity) = collect(keys(sensors(entity)))

get_actuator(emitter::AbstractEntity, recipient) =
    get_actuator(markov_blanket(emitter), recipient)

Rocket.subscribe!(first::AbstractEntity, second::AbstractEntity) =
    throw(MixedStateSpaceException(first, second))

function Rocket.subscribe!(
    emitter::AbstractEntity{T,S,E} where {T,E},
    receiver::AbstractEntity{O,S,P} where {O,P},
) where {S}
    if emitter === receiver
        throw(SelfSubscriptionException(emitter))
    end
    actuator = Actuator()
    insert!(actuators(markov_blanket(emitter)), receiver, actuator)
    add_sensor!(markov_blanket(receiver), emitter, receiver)
    add_to_state!(decorated(emitter), decorated(receiver))
end

function Rocket.subscribe!(emitter::AbstractEntity, receiver::Rocket.Actor{T} where {T})
    actuator = Actuator()
    insert!(actuators(emitter), receiver, actuator)
    return subscribe!(actuator, receiver)
end


function subscribe_to_observations!(entity::AbstractEntity, actor)
    subscribe!(observations(entity), actor)
    return actor
end
"""
Unsubscribes `receiver` from `emitter`. Any data sent from `emitter` to `receiver` will not be received by `receiver` after this function is called.
"""
function Rocket.unsubscribe!(emitter::AbstractEntity, receiver::AbstractEntity)
    Rocket.unsubscribe!(sensors(receiver)[emitter])
    delete!(sensors(receiver), emitter)
    delete!(actuators(emitter), receiver)
end

function Rocket.unsubscribe!(
    emitter::AbstractEntity,
    actor::Rocket.Actor,
    subscription::Teardown,
)
    delete!(actuators(emitter), actor)
    unsubscribe!(subscription)
end

"""
    add!(first::AbstractEntity{T,S,E}, second; environment=false) where {T,S,E}

Adds `second` to `first`. If `environment` is `true`, the `AbstractEntity` created for `second` will be labeled as an environment. 
The Markov Blankets for both entities will be subscribed to each other.

# Arguments
- `first::AbstractEntity{T,S,E}`: The entity to which `second` will be added.
- `second`: The entity to be added to `first`.
- `environment=false`: A boolean indicating whether `second` should be instantiated as an environment.
"""
function add!(first::AbstractEntity{T,S,E}, second; environment = false) where {T,S,E}
    environment = environment ? IsEnvironment() : IsNotEnvironment()
    entity = create_entity(second, state_space(first), environment)
    add!(first, entity)
    return entity
end

function add!(
    first::AbstractEntity{T,S,E} where {T,E},
    second::AbstractEntity{O,S,P} where {O,P},
) where {S}
    subscribe!(first, second)
    subscribe!(second, first)
end

add!(first::AbstractEntity, second::AbstractEntity) =
    throw(MixedStateSpaceException(first, second))

"""
    is_subscribed(subject::AbstractEntity, target::AbstractEntity)

Check if `subject` is subscribed to `target`.

# Arguments
- `subject::AbstractEntity`: The entity that may be subscribed to `target`.
- `target::AbstractEntity`: The entity that may be subscribed to by `subject`.

# Returns
`true` if `subject` is subscribed to `target`, `false` otherwise.
"""
function is_subscribed(subject::AbstractEntity, target::AbstractEntity)
    return haskey(actuators(markov_blanket(target)), subject) &&
           haskey(sensors(markov_blanket(subject)), target)
end

function is_subscribed(subject::Rocket.Actor{Any}, target::AbstractEntity)
    return haskey(actuators(markov_blanket(target)), subject)
end

"""
    terminate!(entity::AbstractEntity)

Terminate an entity by setting its `is_terminated` flag to `true` and severing off all subscriptions to and from the entity.

# Arguments
- `entity::AbstractEntity`: The entity to terminate.

"""
function terminate!(entity::AbstractEntity)
    terminate!(properties(entity).terminated)
    for subscriber in subscribers(entity)
        unsubscribe!(entity, subscriber)
    end
    for subscribed_to in subscribed_to(entity)
        unsubscribe!(subscribed_to, entity)
    end
end

update!(any) =
    @warn "`update!` triggered for entity of type $(typeof(any)), but no update function is defined for this type."
update!(any, elapsed_time) =
    @warn "`update!` triggered for entity of type $(typeof(any)), but no update function is defined for this type."

"""
    update!(e::AbstractEntity{T,ContinuousEntity,E}) where {T,E}

Update the state of the entity `e` based on its current state and the time elapsed since the last update. Acts as state transition function.

# Arguments
- `e::AbstractEntity{T,ContinuousEntity,E}`: The entity to update.
"""
function update!(e::AbstractEntity{T,ContinuousEntity,E}) where {T,E}
    c = clock(e)
    update!(decorated(e), elapsed_time(c))
    set_last_update!(c, time(c))
end

update!(e::AbstractEntity{T,DiscreteEntity,E}) where {T,E} = update!(decorated(e))

"""
    send!(recipient::AbstractEntity, emitter::AbstractEntity, action::Any)

Send an action from `emitter` to `recipient`.
"""
function send!(
    recipient::Union{AbstractEntity,Rocket.Actor},
    emitter::AbstractEntity,
    action::Any,
)
    actuator = get_actuator(emitter, recipient)
    send_action!(actuator, action)
end

what_to_send(recipient::AbstractEntity, emitter::AbstractEntity, observation) =
    what_to_send(decorated(recipient), decorated(emitter), observation)
what_to_send(recipient, emitter::AbstractEntity, observation) =
    what_to_send(recipient, decorated(emitter), observation)
what_to_send(recipient, emitter, observation) = what_to_send(recipient, emitter)
what_to_send(recipient, emitter) = nothing

function receive!(recipient::AbstractEntity, observations::ObservationCollection)
    for observation in observations
        receive!(recipient, observation)
    end
end

"""
    receive!(recipient::AbstractEntity, emitter::AbstractEntity, observation::Any)

Receive an observation from `emitter` and update the state of `recipient` accordingly.

See also: [`RxEnvironments.send!`](@ref)
"""
receive!(recipient::AbstractEntity, observation::Observation) =
    receive!(recipient, emitter(observation), data(observation))
receive!(recipient::AbstractEntity, emitter::AbstractEntity, observation::Any) =
    receive!(decorated(recipient), decorated(emitter), observation)
receive!(recipient::AbstractEntity, observation::Any) = nothing
receive!(recipient, emitter, observation) = nothing

emits(subject::AbstractEntity, listener::AbstractEntity, observation::TimerMessage) =
    emits(decorated(subject), decorated(listener), observation)
emits(subject::AbstractEntity, listener::AbstractEntity, observation::Observation) =
    emits(decorated(subject), decorated(listener), data(observation))

"""
    emits(subject, listener, observation)

Determines if an entity of the type of `subject` should emit an observation to an entity of the type of `listener` when presented with an obervation of type `observation`.
Users should implement this function for their own entity and observation types to handle the logic of when to emit observations to which entities. By default, this function returns `true`.
"""
emits(subject, listener, observation::Any) = true

function add_timer!(
    entity::AbstractEntity{T,ContinuousEntity,E} where {T,E},
    emit_every_ms::Int;
    real_time_factor::Real = 1.0,
)
    @assert real_time_factor > 0.0
    c = Timer(emit_every_ms, entity)
    add_timer!(entity, c)
end

function add_timer!(entity::AbstractEntity{T,ContinuousEntity,E} where {T,E}, timer::Timer)
    properties(entity).timer = timer
end

function animate_state end
function plot_state end
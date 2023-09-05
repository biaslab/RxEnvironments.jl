using Rocket

struct IsEnvironment end
struct IsNotEnvironment end

struct DiscreteEntity end
struct ContinuousEntity end

export AbstractEntity,
    add!,
    act!,
    update!,
    observe,
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

entity(entity::AbstractEntity) = entity.entity
observations(entity::AbstractEntity) = observations(markov_blanket(entity))
actuators(entity::AbstractEntity) = actuators(markov_blanket(entity))
sensors(entity::AbstractEntity) = sensors(markov_blanket(entity))
subscribers(entity::AbstractEntity) = collect(keys(actuators(entity)))
subscribed_to(entity::AbstractEntity) = collect(keys(sensors(entity)))
markov_blanket(entity::AbstractEntity) = entity.markov_blanket
get_actuator(emitter::AbstractEntity, recipient::AbstractEntity) =
    get_actuator(markov_blanket(emitter), recipient)
is_terminated(entity::AbstractEntity) = is_terminated(properties(entity).terminated)
clock(entity::AbstractEntity) = properties(entity).clock
last_update(entity::AbstractEntity{T,ContinuousEntity,E}) where {T,E} =
    last_update(clock(entity))
state_space(entity::AbstractEntity) = properties(entity).state_space



function add!(environment::AbstractEntity{T,S,E}, entity) where {T,S,E}
    entity = create_entity(entity, state_space(environment), IsNotEnvironment())
    add!(environment, entity)
    return entity
end

function add!(first::AbstractEntity{T,S,E}, second::AbstractEntity{O,S,P}) where {T,S,E,O,P}
    subscribe!(first, second)
    subscribe!(second, first)
end

function update!(e::AbstractEntity{T,ContinuousEntity,E}) where {T,E}
    update!(entity(e), elapsed_time(clock(e)))
    set_last_update!(clock(e), time(clock(e)))
end

update!(e::AbstractEntity{T,DiscreteEntity,E}) where {T,E} = update!(entity(e))

function terminate!(entity::AbstractEntity)
    terminate!(properties(entity).terminated)
    for subscriber in subscribers(entity)
        unsubscribe!(entity, subscriber)
    end
    for subscribed_to in subscribed_to(entity)
        unsubscribe!(subscribed_to, entity)
    end
end

observe(subject::AbstractEntity, environment) = observe(entity(subject), environment)
observe(subject, emitter) = nothing

function act!(subject::AbstractEntity, actions::ObservationCollection)
    for observation in actions
        act!(subject, observation)
    end
end

act!(subject::AbstractEntity, action::Observation) =
    act!(subject, emitter(action), data(action))
act!(recipient::AbstractEntity, sender::AbstractEntity, action::Any) =
    act!(entity(recipient), entity(sender), action)
act!(recipient::AbstractEntity, sender::Any, action::Any) =
    act!(entity(recipient), sender, action)
act!(subject::AbstractEntity, action::Any) = nothing
act!(subject, recipient, action) = nothing


function subscribe_to_observations!(entity::AbstractEntity, actor)
    subscribe!(observations(entity), actor)
    return actor
end


function is_subscribed(subject::AbstractEntity, target::AbstractEntity)
    return haskey(actuators(markov_blanket(target)), subject) &&
           haskey(sensors(markov_blanket(subject)), target)
end

function is_subscribed(subject::Rocket.Actor{Any}, target::AbstractEntity)
    return haskey(actuators(markov_blanket(target)), subject)
end

set_clock!(entity::AbstractEntity, clock::Clock) = properties(entity).clock = clock

function add_timer!(
    entity::AbstractEntity{T,ContinuousEntity,E} where {T,E},
    emit_every_ms::Int;
    real_time_factor::Real = 1.0,
)
    @assert real_time_factor > 0.0
    c = Clock(real_time_factor, emit_every_ms)
    add_timer!(entity, c)
end

function add_timer!(entity::AbstractEntity{T,ContinuousEntity,E} where {T,E}, clock::Clock)
    actor = TimerActor(entity)
    subscribe!(clock, actor)
    set_clock!(entity, clock)
end

function animate_state end
function plot_state end
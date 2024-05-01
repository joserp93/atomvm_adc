%%
%% Copyright (c) 2024 Jose Rodriguez
%% All rights reserved.
%%
%% Licensed under the Apache License, Version 2.0 (the "License");
%% you may not use this file except in compliance with the License.
%% You may obtain a copy of the License at
%%
%%     http://www.apache.org/licenses/LICENSE-2.0
%%
%% Unless required by applicable law or agreed to in writing, software
%% distributed under the License is distributed on an "AS IS" BASIS,
%% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
%% See the License for the specific language governing permissions and
%% limitations under the License.
%%
%%-----------------------------------------------------------------------------
%% @doc ADC support.
%%
%% Use this module to take ADC readings on an ESP32 device. ADC1 is enabled by 
%% default and allows taking reading from pins 32-39. If ADC2 is also enabled
%% pins 0, 2, 4, 12-15, and 25-27 may be used as long as WiFi is not required
%% by the application. A solution should be available soon to allow for
%% non-simultaneous use of WiFi and ADC2 channels.
%% @end
%%-----------------------------------------------------------------------------
-module(adc).

-export([
    start/0, start/1, start_link/0, start_link/1, stop/1
]).
-export([
    read/2, read/3, config_width_attenuation/2, config_width_attenuation/3
]).
-export([
    config_calibration/2, config_calibration/3
]).
-export([init/1, handle_call/3, handle_cast/2, handle_info/2, terminate/2, code_change/3]).
-export([nif_init/1, nif_close/1, nif_config_channel_bitwidth_atten/3, nif_config_channel_calibration/3, nif_take_reading/3]). %% internal nif APIs

-behaviour(gen_server).

-include_lib("atomvm_lib/include/trace.hrl").

-type adc_bus() :: pid().
-type adc_peripheral() ::  1 | 2.
-type adc_pin() ::  adc1_pin() | adc2_pin().
-type adc1_pin() :: 32..39.
-type adc2_pin() :: 0 | 2 | 4 | 12..15 | 25..27.
-type options() :: [option()].
-type option_cali() :: [{attenuation, attenuation()}].
-type bit_width() :: bit_9 | bit_10 | bit_11 | bit_12 | bit_13 | bit_max.
-type attenuation() :: db_0 | db_2_5 | db_6 | db_11.
-type option() :: {bit_width, bit_width()} | {attenuation, attenuation()}.

-type read_options() :: [read_option()].
-type read_option() :: raw | voltage | {samples, pos_integer()}.

-type raw_value() :: 0..4095 | undefined.
-type voltage_reading() :: 0..3300 | undefined.
-type reading() :: {raw_value(), voltage_reading()}.

-define(DEFAULT_OPTIONS, [{bit_width, bit_12}, {attenuation, db_11}]).
-define(DEFAULT_OPTIONS_CALI, [{attenuation, db_11}]).
-define(DEFAULT_SAMPLES, 64).
-define(DEFAULT_PERIPHERAL, 1).
-define(DEFAULT_READ_OPTIONS, [raw, voltage, {samples, ?DEFAULT_SAMPLES}]).

-record(state, {
    adc
}).


%%-----------------------------------------------------------------------------
%% @param   Pin     pin from which to read ADC
%% @returns ok | {error, Reason}
%% @equiv   start(Pin, [{bit_width, bit_12}, {attenuation, db_11}])
%% @doc     Start an ADC.
%% @end
%%-----------------------------------------------------------------------------
-spec start() -> {ok, adc_bus()} | {error, Reason::term()}.
start() ->
    start(?DEFAULT_PERIPHERAL).

%%-----------------------------------------------------------------------------
%% @param   Peripheral         Peripheral from which to read ADC
%% @returns {ok, adc_bus()} on success, or {error, Reason}, on failure
%% @doc     Start a ADC.
%%
%% Readings will be taken from the specified pin.  If the pin in not one of the
%% following: 32..39 (and 0|2|4|12..15|25..27 with adc2 enabled), a badarg
%% exception will be raised.
%%
%% Options may specify the bit width and attenuation. The attenuation value `bit_max'
%% may be used to automatically select the highest sample rate supported by your
%% ESP chip-set.
%%
%% Note. Unlike the esp-idf adc driver bit widths are used on a per pin basis,
%% so pins on the same adc unit can use different widths if necessary.
%%
%% Use the returned reference in subsequent ADC operations.
%% @end
%%-----------------------------------------------------------------------------
-spec start(Peripheral::adc_peripheral()) -> {ok, adc_bus()} | {error, Reason::term()}.
start(Peripheral) ->
    gen_server:start(?MODULE, Peripheral, []).

%%-----------------------------------------------------------------------------
%% @param   Options
%% @returns {ok, adc_bus()} on success, or {error, Reason}, on failure
%% @doc     Start the ADC Bus.
%% @end
%%-----------------------------------------------------------------------------
-spec start_link() -> {ok, adc_bus()} | {error, Reason::term()}.
start_link() ->
    gen_server:start_link(?MODULE, ?DEFAULT_PERIPHERAL, []).

%%-----------------------------------------------------------------------------
%% @param   Options
%% @returns {ok, adc_bus()} on success, or {error, Reason}, on failure
%% @doc     Start the ADC Bus.
%% @end
%%-----------------------------------------------------------------------------
-spec start_link(Peripheral::adc_peripheral()) -> {ok, adc_bus()} | {error, Reason::term()}.
start_link(Peripheral) ->
    gen_server:start_link(?MODULE, Peripheral, []).

%%-----------------------------------------------------------------------------
%% @returns ok
%% @doc     Stop the specified ADC.
%% @end
%%-----------------------------------------------------------------------------
-spec stop(Bus::adc_bus()) -> ok.
stop(Bus) ->
    gen_server:stop(Bus).

%%-----------------------------------------------------------------------------
%% @param   Pin         pin from which to read ADC
%% @returns {ok, {RawValue, MilliVoltage}} | {error, Reason}
%% @equiv   read(ADC, [raw, voltage, {samples, 64}])
%% @doc     Take a reading from the pin associated with this ADC.
%%
%% @end
%%-----------------------------------------------------------------------------
-spec read(Bus::adc_bus(), Pin::adc_pin()) -> {ok, reading()} | {error, Reason::term()}.
read(Bus, Pin) ->
    read(Bus, Pin, ?DEFAULT_READ_OPTIONS).

%%-----------------------------------------------------------------------------
%% @param   Pin         pin from which to read ADC
%% @param   ReadOptions extra options
%% @returns {ok, {RawValue, MilliVoltage}} | {error, Reason}
%% @doc     Take a reading from the pin associated with this ADC.
%%
%% The Options parameter may be used to specify the behavior of the read
%% operation.
%%
%% If the ReadOptions contains the atom `raw', then the raw value will be returned
%% in the first element of the returned tuple.  Otherwise, this element will be the
%% atom `undefined'.
%%
%% If the ReadOptions contains the atom `voltage', then the millivoltage value will be returned
%% in the second element of the returned tuple.  Otherwise, this element will be the
%% atom `undefined'.
%%
%% You may specify the number of samples to be taken and averaged over using the tuple
%% `{samples, Samples::pos_integer()}'.
%%
%% If the error `Reason' is timeout and the adc channel is on unit 2 then WiFi is likely
%% enabled and adc2 readings will no longer be possible.
%% @end
%%-----------------------------------------------------------------------------
-spec read(Bus::adc_bus(), Pin::adc_pin(), ReadOptions::read_options()) -> {ok, reading()} | {error, Reason::term()}.
read(Bus, Pin, ReadOptions) ->
    gen_server:call(Bus, {read, Pin, ReadOptions}).

-spec config_calibration(Bus::adc_bus(), Pin::adc_pin()) -> ok | {error, Reason::term()}.
config_calibration(Bus, Pin) ->
    config_calibration(Bus, Pin, ?DEFAULT_OPTIONS_CALI).

-spec config_calibration(Bus::adc_bus(), Pin::adc_pin(), Options::option_cali()) -> ok | {error, Reason::term()}.
config_calibration(Bus, Pin, Options) ->
    gen_server:call(Bus, {calibration, Pin, Options}).

-spec config_width_attenuation(Bus::adc_bus(), Pin::adc_pin()) -> ok | {error, Reason::term()}.
config_width_attenuation(Bus, Pin) ->
    config_width_attenuation(Bus, Pin, ?DEFAULT_OPTIONS).

-spec config_width_attenuation(Bus::adc_bus(), Pin::adc_pin(), Options::options()) -> ok | {error, Reason::term()}.
config_width_attenuation(Bus, Pin, Options) ->
    gen_server:call(Bus, {config, Pin, Options}).


%%
%% gen_server API
%%

%% @hidden
init(Peripheral) ->
    ADC = adc:init(Peripheral),
    ?TRACE("ADC opened. ADC_UNIT: ~p", [Peripheral]),
    State = #state{
        adc = ADC
    },
    {ok, State}.

%% @hidden
handle_call({read, Pin, ReadOptions}, _From, State) ->
    Reading = adc:take_reading(State#state.adc, Pin, ReadOptions),
    ?TRACE("Reply: ~p", [Reading]),
    {reply, {ok, Reading}, State};
handle_call({config, Pin, Options}, _From, State) ->
    Reply = adc:config_channel_bitwidth_atten(State#state.adc, Pin, Options),
    ?TRACE("Reply: ~p", [Reply]),
    {reply, Reply, State};
handle_call({calibration, Pin, Options}, _From, State) ->
    Reply = adc:config_channel_calibration(State#state.adc, Pin, Options),
    ?TRACE("Reply: ~p", [Reply]),
    {reply, Reply, State};
handle_call(Request, _From, State) ->
    {reply, {error, {unknown_request, Request}}, State}.

%% @hidden
handle_cast(_Msg, State) ->
    {noreply, State}.

%% @hidden
handle_info(_Info, State) ->
    {noreply, State}.

%% @hidden
terminate(_Reason, State) ->
    io:format("Closing ADC ... ~n"),
    adc:close(State#state.adc),
    ok.

%% @hidden
code_change(_OldVsn, State, _Extra) ->
    {ok, State}.

%%
%% internal nif API operations
%%



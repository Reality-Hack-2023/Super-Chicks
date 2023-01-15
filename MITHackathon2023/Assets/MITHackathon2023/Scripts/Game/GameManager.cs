using System;
using System.Collections.Generic;
using MITHack.Robot.Entities;
using MITHack.Robot.Utils;
using UnityEngine;

namespace MITHack.Robot.Game
{
    public class GameManager : GenericSingleton<GameManager>
    {
        #region defines
        
        public struct StatisticChangeEventContext<TStatistic>
            where TStatistic : unmanaged
        {
            public TStatistic prev;
            public TStatistic next;
        }
        
        public delegate void GameManagerGenericDelegate<in TContext>(TContext context);
        
        #endregion

        public GameManagerGenericDelegate<StatisticChangeEventContext<int>> LivesChangedEvent;

        [Header("Statistics")]
        [SerializeField, Min(1)]
        private int totalLives = 3;
        [Space] 
        [SerializeField, Min(0.0f)]
        private float timeBetweenEachLife = 2.0f;

        [Header("References")] 
        [SerializeField]
        private List<Spawner.Spawner> spawners;

        private int _chickensKilled = 0;
        private int _currentLives = 0;
        private float _currentAwaitingTime = 0.0f;

        private bool _spawnersEnabled = true;

        private Action<RobotEntity.ChickenKilledContext> _onChickenKilled;
        private RobotEntity.RobotEntityGenericDelegate<RobotEntity.RobotEntityStateChangeContext> _onStateChange;

        private RobotEntity RobotEntity => RobotEntity.Get<RobotEntity>();

        public bool SpawnersEnabled => _spawnersEnabled;
        
        /// <summary>
        /// The current amount of lives.
        /// </summary>
        public int ChickensKilled => _chickensKilled;

        /// <summary>
        /// The total chickens killed.
        /// </summary>
        public int CurrentLives => _currentLives;

        public int TotalLives => totalLives;

        protected override void Awake()
        {
            base.Awake();

            _onStateChange = OnRobotStateChange;
            _onChickenKilled = OnChickenKilled;
            
            _chickensKilled = 0;
            _currentLives = totalLives;
            SetSpawnersEnabled(true);
        }

        private void OnEnable()
        {
            if (RobotEntity)
            {
                RobotEntity.StateChangedEvent += _onStateChange;
                RobotEntity.ChickenKilledEvent += _onChickenKilled;
            }
        }

        private void OnDisable()
        {
            if (RobotEntity)
            {
                RobotEntity.StateChangedEvent -= _onStateChange;
                RobotEntity.ChickenKilledEvent -= _onChickenKilled;
            }
        }

        private void Update()
        {
            if (_currentAwaitingTime > 0.0f)
            {
                _currentAwaitingTime -= Time.deltaTime;
                if (_currentAwaitingTime <= 0.0f)
                {
                    ResumeGameOnLifeLost();
                }
            }
        }
        
        private void OnChickenKilled(RobotEntity.ChickenKilledContext obj)
        {
            _chickensKilled++;
        }
        
        private void AddLives(int lives)
        {
            SetCurrentLives(_currentLives + lives);
        }

        private void RemoveLives(int lives)
        {
            SetCurrentLives(_currentLives - lives);
        }
        
        private void SetCurrentLives(int currentLives)
        {
            if (_currentLives != currentLives)
            {
                LivesChangedEvent?.Invoke(new StatisticChangeEventContext<int>()
                {
                    prev = _currentLives,
                    next = currentLives
                });
            }
            _currentLives = currentLives;
        }

        private void SetSpawnersEnabled(bool spawnersEnabled)
        {
            SetSpawnersEnabled(spawnersEnabled,  false);
        }

        private void SetSpawnersEnabled(bool spawnersEnabled, bool force)
        {
            if (!force && spawnersEnabled == _spawnersEnabled) return;
            foreach (var spawner in spawners)
            {
                if (spawner) spawner.enabled = spawnersEnabled;
            }
            _spawnersEnabled = spawnersEnabled;
        }


        private void OnRobotStateChange(RobotEntity.RobotEntityStateChangeContext context)
        {
            if (context.next == RobotEntity.RobotEntityState.StateDead)
            {
                StopGameOnLifeLost();
            }
        }

        private void ResumeGameOnLifeLost()
        {
            SetSpawnersEnabled(true);
            _currentAwaitingTime = 0.0f;
        }
        
        private void StopGameOnLifeLost()
        {
            RemoveLives(1);
            SetSpawnersEnabled(false);
            _currentAwaitingTime = timeBetweenEachLife;
        }
    }
}
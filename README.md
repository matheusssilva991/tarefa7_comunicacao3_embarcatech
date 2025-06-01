# Estacionamento Inteligente - Pico W

Este projeto implementa um sistema de controle de estacionamento utilizando a Raspberry Pi Pico W, com integração MQTT, matriz de LEDs WS2812B, display OLED SSD1306 e buzzer. O sistema permite monitorar e reservar vagas remotamente via MQTT, além de indicar o status visualmente e sonoramente.

## Funcionalidades

- **Controle de vagas:** Indica vagas livres, ocupadas e reservadas.
- **Reserva remota:** Recebe comandos de reserva via MQTT.
- **Indicação visual:** Matriz de LEDs mostra o status de cada vaga.
- **Indicação sonora:** Buzzer sinaliza mudanças de status.
- **Display OLED:** Mostra o status de todas as vagas.
- **Botões físicos:** Permite navegação e alteração de status localmente.
- **Publicação periódica:** Publica o status das vagas no MQTT a cada 10 segundos.
- **Expiração automática de reservas:** Reservas expiram após 10 segundos.

## Hardware

- Raspberry Pi Pico W
- Matriz de LEDs WS2812B (NeoPixel)
- Display OLED SSD1306 (I2C)
- Buzzer
- Botões (A, B, SW)
- Fonte de alimentação adequada

## Instalação

1. **Clone o repositório:**
    ```sh
    git clone https://github.com/matheusssilva991/tarefa7_comunicacao3_embarcatech.git
    cd tarefa7_comunicacao3_embarcatech
    ```

2. **Configure as credenciais Wi-Fi e MQTT:**
    - Edite o arquivo `config/credential_config.h` com seu SSID, senha e dados do broker MQTT.

3. **Compile o projeto:**
    ```sh
    mkdir build
    cd build
    cmake -G Ninja ..
    ninja
    ```

4. **Grave o firmware na Pico W.**

## Uso

- O sistema conecta-se automaticamente ao Wi-Fi e ao broker MQTT.
- O status das vagas é publicado periodicamente em tópicos como `/parking/status/1`, `/parking/status/2`, etc.
- Para reservar uma vaga remotamente, publique uma mensagem em `/parking/{id}/reservation` (ex: `/parking/1/reservation`).
- Reservas expiram automaticamente após 10 segundos.
- O display OLED mostra o status de todas as vagas.
- Os botões permitem navegar entre vagas e alterar o status manualmente.

## Vídeo de Demonstração

[Assista aqui](https://drive.google.com/file/d/1gPM2zoX-GFib4uM49fF6U3Q7-8LDpwXU/view?usp=drive_link)

## Tópicos MQTT

- **Publicação de status:**
  `/parking/status/{id}`
  Payload: `0` (livre), `1` (ocupada), `2` (reservada)

- **Reserva remota:**
  `/parking/{id}/reservation`
  Payload: qualquer valor (reserva a vaga se estiver livre)

## Estrutura do Código

- `src/main.c`: Lógica principal do sistema.
- `lib/`: Bibliotecas auxiliares (botão, LED, display, buzzer).
- `config/credential_config.h`: Configurações de Wi-Fi e MQTT.



## Licença

MIT

---
# Humanoid Robotics Book with RAG Integration

Welcome to the Humanoid Robotics Book project! This is a comprehensive educational resource covering ROS 2, Digital Twins, NVIDIA Isaac, and Vision-Language-Action (VLA) paradigms for humanoid robotics, enhanced with a Retrieval-Augmented Generation (RAG) AI assistant.

## 📚 About This Book

This book is designed to guide you through the fundamentals of humanoid robotics, from ROS 2 communication patterns to advanced AI perception and control systems. Each module builds upon the previous one, creating a complete learning path for robotics enthusiasts and professionals.

## 🤖 AI-Powered Documentation Assistant

This book features an integrated AI assistant powered by Retrieval-Augmented Generation (RAG) technology that can help you understand complex concepts and find relevant information:

- **Context-Aware Q&A**: Ask questions about any topic in the documentation and get AI-generated answers with citations
- **Text Selection Integration**: Select any text on a page and ask specific questions about it
- **Source Citations**: All answers include links to the original documentation for verification
- **24/7 Availability**: Get help anytime without waiting for community responses

## 🚀 Local Development Setup

### Prerequisites

- [Node.js](https://nodejs.org/) (version 18 or higher)
- [npm](https://www.npmjs.com/) or [yarn](https://yarnpkg.com/)

### Environment Configuration

To run the RAG integration locally, you'll need to set up your environment variables:

1. Copy `.env.example` to `.env.local`:
   ```bash
   cp .env.example .env.local
   ```

2. Update `.env.local` with your OpenAI API key and other required configuration

### Running with RAG Integration

1. Start the backend server (in a separate terminal):
   ```bash
   cd backend
   pip install -r requirements.txt
   python -m uvicorn app.main:app --reload --port 8000
   ```

2. In another terminal, start the Docusaurus frontend:
   ```bash
   npm start
   ```

The AI assistant will be available on the documentation pages and the dedicated chat page at `/chatbot`.

### Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/your-username/humanoid-robotic-book.git
   cd humanoid-robotic-book
   ```

2. Install dependencies:
   ```bash
   npm install
   ```

### Running the Development Server

To start the development server and open the documentation in your browser:

```bash
npm start
```

This will start the Docusaurus development server and open your site at `http://localhost:3000/humanoid-robotic-book/`.

### Building the Site

To build the static site for production:

```bash
npm run build
```

The built site will be available in the `build/` directory.

### Linting

To lint the markdown files:

```bash
npm run lint
```

To automatically fix linting issues:

```bash
npm run lint:fix
```

## 🤖 Using the AI Assistant

### On Documentation Pages
- Type questions in the chat box that appears on each documentation page
- Select text on any page and click the floating "Ask" button to ask questions about the selected text
- View source citations to verify information and explore related topics

### On the Chat Page
- Visit the `/chatbot` page for a dedicated chat interface
- Ask questions about the entire documentation corpus
- Browse your conversation history

## 📖 Project Structure

- `docs/chapters/` - Main book chapters organized by module
- `docs/appendices/` - Additional resources and reference materials
- `docs/assets/` - Diagrams, code snippets, and other assets
- `src/` - Custom React components and styling
- `src/api/` - API client for RAG backend communication
- `src/components/ChatBox/` - Main chat interface component
- `src/components/TextSelectionChat/` - Text selection integration component
- `src/theme/` - Docusaurus theme components
- `docusaurus.config.js` - Site configuration
- `sidebars.js` - Navigation structure

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Make your changes
4. Add and commit your changes (`git add . && git commit -m 'Add amazing feature'`)
5. Push to your fork (`git push origin feature/amazing-feature`)
6. Open a Pull Request

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🎯 Modules

1. **Module 1: ROS 2 — Robotic Nervous System** - Learn ROS 2 fundamentals, nodes, topics, services, and URDF
2. **Module 2: Gazebo + Unity — Digital Twin** - Create simulation environments with Gazebo and Unity
3. **Module 3: NVIDIA Isaac — AI Brain & Perception** - Implement AI perception using NVIDIA Isaac
4. **Module 4: VLA — Vision, Language, Action** - Develop multimodal autonomous systems